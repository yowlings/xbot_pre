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
     float power_voltage;
     uint16_t infred_1;
     u_int16_t infred_2;
     float current_1;
     float current_2;
     float current_3;
     float echo_1;
     float echo_2;
     float echo_3;
     float echo_4;
     uint16_t left_encoder;
     uint16_t right_encoder;
     uint16_t up_encoder;
     uint8_t charger;
     uint8_t battery;
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
