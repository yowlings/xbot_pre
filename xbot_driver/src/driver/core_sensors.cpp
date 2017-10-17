/**
 * @file /xbot_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/core_sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"
#include <time.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  buildBytes(Header::CoreSensors, byteStream);
//  buildBytes(length, byteStream);
//  buildBytes(data.time_stamp, byteStream);	//2
//  buildBytes(data.bumper, byteStream);		//1
//  buildBytes(data.wheel_drop, byteStream);	//1
//  buildBytes(data.cliff, byteStream);		//1
//  buildBytes(data.left_encoder, byteStream);	//2
//  buildBytes(data.right_encoder, byteStream);	//2
//  buildBytes(data.left_pwm, byteStream);	//1
//  buildBytes(data.right_pwm, byteStream);	//1
//  buildBytes(data.buttons, byteStream);		//1
//  buildBytes(data.charger, byteStream);		//1
//  buildBytes(data.battery, byteStream);		//1
//  buildBytes(data.over_current, byteStream);	//1

  return true;
}



bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//    std::cout<<byteStream.size()<<std::endl;

//  if (byteStream.size() < length+2)
//  {
//    std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }

  unsigned char header_id;
  buildVariable(header_id, byteStream);
//  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;

  if( header_id != Header::CoreSensors ) return false;
  unsigned char power_num;
  buildVariable(power_num, byteStream);
//  std::cout<<"power_num:"<<(unsigned int)power_num<<std::endl;

  build_special_variable(data.power_voltage,byteStream);
//  std::cout<<"power_voltage:"<<data.power_voltage<<std::endl;
  unsigned char infred_num;
  buildVariable(infred_num, byteStream);
  buildVariable(data.infred_1,byteStream);
  buildVariable(data.infred_2,byteStream);

  unsigned char current_num;
  buildVariable(current_num,byteStream);
  build_special_variable(data.current_1,byteStream);
  build_special_variable(data.current_2,byteStream);
  build_special_variable(data.current_3,byteStream);

  unsigned char echo_num;
  buildVariable(echo_num,byteStream);
  build_special_variable(data.echo_1,byteStream);
  build_special_variable(data.echo_2,byteStream);
  build_special_variable(data.echo_3,byteStream);
  build_special_variable(data.echo_4,byteStream);

  unsigned char encoder_num;
  buildVariable(encoder_num, byteStream);
  buildVariable(data.left_encoder, byteStream);
  buildVariable(data.right_encoder, byteStream);
  buildVariable(data.up_encoder, byteStream);

  std::cout<<"time:"<<time(0)<<"|left_encoder:"<<data.left_encoder<<std::endl;


//  std::cout<<"power:"<<data.power_voltage<<"|Echo1:"<<data.echo_1<<"|Echo2:"<<data.echo_2<<"|Echo3:"<<data.echo_3<<"|Echo4:"<<data.echo_4<<std::endl;
  return true;
}

void CoreSensors::build_special_variable(float &variable, ecl::PushAndPop<unsigned char> &byteStream)
{
    if (byteStream.size() < 2)
      return;

    unsigned char a, b;
    buildVariable(a,byteStream);
    buildVariable(b,byteStream);
    variable = ((unsigned int)(a&0x0f))/100.0;

    variable += ((unsigned int)(a>>4))/10.0;

    variable += (unsigned int)(b&0x0f);

    variable += ((unsigned int)(b>>4))*10;


}


} // namespace xbot
