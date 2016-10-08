/**
 * @file /xbot_driver/include/xbot_driver/modules/diff_drive.hpp
 *
 * @brief Simple module for the diff drive odometry.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_DIFF_DRIVE_HPP_
#define XBOT_DIFF_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <climits>
#include <stdint.h>
#include <ecl/mobile_robot.hpp>
#include <ecl/threads/mutex.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class xbot_PUBLIC DiffDrive {
public:
  DiffDrive();
  const ecl::DifferentialDrive::Kinematics& kinematics() { return diff_drive_kinematics; }
  void update(const unsigned int &time_stamp,
              const uint16_t &left_encoder,
              const uint16_t &right_encoder,
              ecl::Pose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);
  void reset();
  void getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate,
                           float &wheel_right_angle, float &wheel_right_angle_rate);
  void setVelocityCommands(const float &vx, const float &wz);
  void velocityCommands(const float &vx, const float &wz);
  void velocityCommands(const std::vector<float> &cmd) { velocityCommands(cmd[0], cmd[1]); }

  /*********************
  ** Command Accessors
  **********************/
  std::vector<float> velocityCommands(); // (speed, radius), in [mm/s] and [mm]
  std::vector<float> pointVelocity() const; // (vx, wz), in [m/s] and [rad/s]

  /*********************
  ** Property Accessors
  **********************/
  float wheel_bias() const { return bias; }

private:
  unsigned int last_timestamp;
  float last_velocity_left, last_velocity_right;
  float last_diff_time;

  unsigned short last_tick_left, last_tick_right;
  float last_rad_left, last_rad_right;

  //float v, w; // in [m/s] and [rad/s]
  std::vector<float> point_velocity; // (vx, wz), in [m/s] and [rad/s]
  float angular_velocity; // in [m/s]
  float linear_velocity;  // in [rad/s]
  float bias; //wheelbase, wheel_to_wheel, in [m]
  float wheel_radius; // in [m]
  int imu_heading_offset;
  const float tick_to_rad;

  ecl::DifferentialDrive::Kinematics diff_drive_kinematics;
  ecl::Mutex velocity_mutex, state_mutex;

  // Utility
  short bound(const float &value);
};

} // namespace xbot

#endif /* XBOT_DIFF_DRIVE_HPP_ */
