/**
 * @file include/xbot_driver/packets/core_sensors.hpp
 *
 * @brief Imu sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_IMU_SENSORS_HPP__
#define XBOT_IMU_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../macros.hpp"
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot
{

/*****************************************************************************
** Interface
*****************************************************************************/

class xbot_PUBLIC ImuSensors : public packet_handler::payloadBase
{
public:
  ImuSensors() : packet_handler::payloadBase(false, 32) {};

  struct Data {
    uint16_t acce_x;
    uint16_t acce_y;
    uint16_t acce_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;
    float pressure;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    unsigned int timestamp;

  } data;


  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace xbot

#endif /* XBOT_IMU_SENSORS_HPP__ */
