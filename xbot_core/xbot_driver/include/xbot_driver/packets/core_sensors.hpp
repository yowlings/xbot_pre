/**
 * @file include/xbot_driver/packets/core_sensors.hpp
 *
 * @brief Core sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_CORE_SENSORS_HPP__
#define XBOT_CORE_SENSORS_HPP__

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

class xbot_PUBLIC CoreSensors : public packet_handler::payloadBase
{
public:
  CoreSensors() : packet_handler::payloadBase(false, 42) {};

  struct Data {
     float battery_voltage;
     uint16_t rear_left_infred;
     uint16_t rear_center_infred;
     uint16_t rear_right_infred;
     float front_left_current;
     float front_right_current;
     float rear_left_current;
     float rear_right_current;
     float up_down_current;
     float front_left_echo;
     float front_center_echo;
     float front_right_echo;
     uint16_t front_left_encoder;
     uint16_t front_right_encoder;
     uint16_t rear_left_encoder;
     uint16_t rear_right_encoder;
     uint16_t up_down_encoder;
     short acce_x;
     short acce_y;
     short acce_z;
     short gyro_x;
     short gyro_y;
     short gyro_z;
     short mag_x;
     short mag_y;
     short mag_z;
     float pressure;
     short yaw;
     short pitch;
     short roll;
     unsigned short timestamp;
  } data;

  struct Flags {
      // Charging source
      // - first four bits distinguish between adapter or docking base charging
      static const uint8_t AdapterType  = 0x10;
      // - last 4 bits specified the charging status (see Battery.hpp for details)
      static const uint8_t BatteryStateMask = 0x0F;
      static const uint8_t Discharging  = 0x00;
      static const uint8_t Charged      = 0x02;
      static const uint8_t Charging     = 0x06;


    };


  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
  void build_special_variable(float &variable, ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace xbot

#endif /* XBOT_CORE_SENSORS_HPP__ */
