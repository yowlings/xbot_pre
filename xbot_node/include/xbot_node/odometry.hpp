/**
 * @file /xbot_node/include/xbot_node/odometry.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_NODE_ODOMETRY_HPP_
#define XBOT_NODE_ODOMETRY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ecl/geometry/pose2d.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Odometry for the xbot node.
 **/
class Odometry {
public:
  Odometry();
  void init(ros::NodeHandle& nh, const std::string& name);
  bool commandTimeout() const;
  void update(const ecl::Pose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
              double imu_heading, double imu_angular_velocity);
  void resetOdometry() { pose.setIdentity(); }
  const ros::Duration& timeout() const { return cmd_vel_timeout; }
  void resetTimeout() { last_cmd_time = ros::Time::now(); }

private:
  geometry_msgs::TransformStamped odom_trans;
  ecl::Pose2D<double> pose;
  std::string odom_frame;
  std::string base_frame;
  ros::Duration cmd_vel_timeout;
  ros::Time last_cmd_time;
  bool publish_tf;
  bool use_imu_heading;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_publisher;

  void publishTransform(const geometry_msgs::Quaternion &odom_quat);
  void publishOdometry(const geometry_msgs::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates);
};

} // namespace xbot

#endif /* XBOT_NODE_ODOMETRY_HPP_ */
