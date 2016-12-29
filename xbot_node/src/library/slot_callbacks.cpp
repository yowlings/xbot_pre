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
 * @file src/node/slot_callbacks.cpp
 *
 * @brief All the slot callbacks for interrupts from the xbot driver.
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "xbot_node/xbot_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

void XbotRos::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishDockIRData();
  publishInertia();
  publishRawInertia();
  publishDebugSensors();
  publishRobotState();

}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void XbotRos::publishSensorState()
{
  if ( ros::ok() ) {
//      float heading = xbot.getHeading();

//      ROS_ERROR("%f",heading);
    if (sensor_state_publisher.getNumSubscribers() > 0) {
      xbot_msgs::SensorState state;
      CoreSensors::Data data = xbot.getCoreSensorData();
      state.header.stamp = ros::Time::now();
      state.left_encoder = data.left_encoder;
      state.right_encoder = data.right_encoder;
      state.charger = data.charger;
      state.battery = data.battery;




      sensor_state_publisher.publish(state);
    }
  }
}

void XbotRos::publishWheelState()
{

//     Take latest encoders and gyro data
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    xbot.updateOdometry(pose_update, pose_update_rates);
    float left_joint_pos,left_joint_vel,right_joint_pos,right_joint_vel;
    xbot.getWheelJointStates(left_joint_pos,left_joint_vel,right_joint_pos,right_joint_vel);  // right wheel
    joint_states.position[0]=(double)left_joint_pos;
    joint_states.velocity[0]=left_joint_vel;
    joint_states.position[1]=(double)right_joint_pos;
//    ROS_ERROR_STREAM("jointstates.position[0]:" << right_joint_pos);

    joint_states.velocity[1]=right_joint_vel;
//    // Update and publish odometry and joint states
//        ROS_ERROR_STREAM("imu_heading:" << xbot.getHeading());
    odometry.update(pose_update, pose_update_rates, xbot.getHeading(), xbot.getAngularVelocity());

    if (ros::ok())
    {
      joint_states.header.stamp = ros::Time::now();
      joint_state_publisher.publish(joint_states);

    }
}

void XbotRos::publishInertia()
{
  if (ros::ok())
  {
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.frame_id = "gyro_link";
      msg->header.stamp = ros::Time::now();

      msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, xbot.getHeading());


      // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
      // set yaw covariance as very low, to make it dominate over the odometry heading when combined
      // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
      msg->orientation_covariance[0] = DBL_MAX;
      msg->orientation_covariance[4] = DBL_MAX;
      msg->orientation_covariance[8] = 0.05;

      // fill angular velocity; we ignore acceleration for now
      msg->angular_velocity.z = xbot.getAngularVelocity();

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg->angular_velocity_covariance[0] = DBL_MAX;
      msg->angular_velocity_covariance[4] = DBL_MAX;
      msg->angular_velocity_covariance[8] = 0.05;

      imu_data_publisher.publish(msg);
    }
  }
}

void XbotRos::publishRawInertia()
{
  if ( ros::ok() && (raw_imu_data_publisher.getNumSubscribers() > 0) )
  {
    // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
    xbot_msgs::ImuNinePtr msg(new xbot_msgs::ImuNine);
    msg->accex = xbot.getImuSensorData().acce_x;
    msg->gyrox = xbot.getImuSensorData().gyro_x;
    msg->magx = xbot.getImuSensorData().mag_x;
    msg->accey = xbot.getImuSensorData().acce_y;
    msg->gyroy = xbot.getImuSensorData().gyro_y;
    msg->magy = xbot.getImuSensorData().mag_y;
    msg->accez = xbot.getImuSensorData().acce_z;
    msg->gyroz = xbot.getImuSensorData().gyro_z;
    msg->magz = xbot.getImuSensorData().mag_z;

    raw_imu_data_publisher.publish(msg);
  }
}

void XbotRos::publishDebugSensors()
{
//    ros::Rate r(50);
    if ( ros::ok() && (debug_sensors_publisher.getNumSubscribers() > 0) )
    {
        xbot_msgs::DebugSensorPtr msg(new xbot_msgs::DebugSensor);
        CoreSensors::Data data_debug = xbot.getCoreSensorData();

        msg->header.frame_id = "encoder";
        msg->header.stamp = ros::Time::now();
        msg->data.push_back(data_debug.left_encoder);
        msg->data.push_back(data_debug.right_encoder);
        msg->heading=xbot.getHeading();
        debug_sensors_publisher.publish(msg);
//        r.sleep();

    }

}

void XbotRos::publishRobotState()
{
    ros::Rate r(10);
    if ( ros::ok() && (robot_state_publisher.getNumSubscribers() > 0) )
    {
        xbot_msgs::XbotStatePtr msg(new xbot_msgs::XbotState);
        CoreSensors::Data data = xbot.getCoreSensorData();

        msg->power = data.power_voltage;
        msg->height_percent = xbot.getHeightPercent();
        msg->platform_degree = xbot.getPlatformDegree();
        msg->camera_degree = xbot.getCameraDegree();

        robot_state_publisher.publish(msg);
        r.sleep();

    }

}

void XbotRos::publishDockIRData()
{
    if (ros::ok())
    {
        if (dock_ir_publisher.getNumSubscribers() > 0)
        {

            // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
            xbot_msgs::DockInfraRedPtr msg(new xbot_msgs::DockInfraRed);
            CoreSensors::Data data_echo = xbot.getCoreSensorData();
            msg->header.frame_id = "dock_ir_link";
            msg->header.stamp = ros::Time::now();

            if(data_echo.echo_1<0.1)
            {
                msg->left_near = true;
            }
            else
            {
                msg->left_near = false;
            }
            if(data_echo.echo_2<0.1)
            {
                msg->center_near = true;
            }
            else
            {
                msg->center_near = false;
            }
            if(data_echo.echo_3<0.1)
            {
               msg->right_near = true;
            }
            else
            {
                msg->right_near = false;
            }



            dock_ir_publisher.publish(msg);
        }
    }
}

/*****************************************************************************
** Non Default Stream Packets
*****************************************************************************/
/**
 * @brief Publish fw, hw, sw version information.
 *
 * The driver will only gather this data when initialising so it is
 * important that this publisher is latched.
 */

void XbotRos::publishControllerInfo()
{
  if (ros::ok())
  {
    xbot_msgs::ControllerInfoPtr msg(new xbot_msgs::ControllerInfo);


    controller_info_publisher.publish(msg);
  }
}

/*****************************************************************************
** Events
*****************************************************************************/








/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data commands. Useful for debugging command to protocol
 * byte packets to the firmware.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void XbotRos::publishRawDataCommand(Command::Buffer &buffer)
{
  if ( raw_data_command_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    std::ostringstream ostream;
    Command::Buffer::Formatter format;
    ostream << format(buffer); // convert to an easily readable hex string.
    std_msgs::String s;
    s.data = ostream.str();
    if (ros::ok())
    {
      raw_data_command_publisher.publish(s);
    }
  }
}
/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data (incoming) stream. Useful for checking when bytes get
 * mangled.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void XbotRos::publishRawDataStream(PacketFinder::BufferType &buffer)
{
  if ( raw_data_stream_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    /*std::cout << "size: [" << buffer.size() << "], asize: [" << buffer.asize() << "]" << std::endl;
    std::cout << "leader: " << buffer.leader << ", follower: " << buffer.follower  << std::endl;
    {
      std::ostringstream ostream;
      PacketFinder::BufferType::Formatter format;
      ostream << format(buffer); // convert to an easily readable hex string.
      //std::cout << ostream.str() << std::endl;
      std_msgs::String s;
      s.data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(s);
      }
    }*/
    {
      std::ostringstream ostream;
      ostream << "{ " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < buffer.size(); i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
      ostream << "}";
      //std::cout << ostream.str() << std::endl;
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(msg);
      }
    }
  }
}

void XbotRos::publishRawControlCommand(const std::vector<short> &velocity_commands)
{
  if ( raw_control_command_publisher.getNumSubscribers() > 0 ) {
    std_msgs::Int16MultiArrayPtr msg(new std_msgs::Int16MultiArray);
    msg->data = velocity_commands;
    if (ros::ok())
    {
      raw_control_command_publisher.publish(msg);
    }
  }
  return;
}

} // namespace xbot
