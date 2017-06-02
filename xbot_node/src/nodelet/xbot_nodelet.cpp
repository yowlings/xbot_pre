/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /xbot_node/src/nodelet/xbot_nodelet.cpp
 *
 * @brief Implementation for the ROS Xbot nodelet
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "xbot_node/xbot_ros.hpp"


namespace xbot
{

class XbotNodelet : public nodelet::Nodelet
{
public:
  XbotNodelet() : shutdown_requested_(false) {};
  ~XbotNodelet()
  {
    NODELET_DEBUG_STREAM("Xbot : waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }
  virtual void onInit()
  {
    NODELET_DEBUG_STREAM("Xbot : initialising nodelet...");
    std::string nodelet_name = this->getName();
    xbot_.reset(new XbotRos(nodelet_name));
    // if there are latency issues with callbacks, we might want to move to process callbacks in multiple threads (use MTPrivateNodeHandle)
    if (xbot_->init(this->getPrivateNodeHandle()))
    {
      update_thread_.start(&XbotNodelet::update, *this);
      NODELET_INFO_STREAM("Xbot : initialised.");
    }
    else
    {
      NODELET_ERROR_STREAM("Xbot : could not initialise! Please restart.");
    }
  }
private:
  void update()
  {
    ros::Rate spin_rate(10);//check the state every 0.1sec
    while (!shutdown_requested_ && ros::ok() && xbot_->update())
    {
      spin_rate.sleep();
    }
  }

  boost::shared_ptr<XbotRos> xbot_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace xbot

PLUGINLIB_EXPORT_CLASS(xbot::XbotNodelet, nodelet::Nodelet);
