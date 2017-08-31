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
  unsigned char data_type;//0x10 marker
  buildVariable(data_type, byteStream);

//  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;
  unsigned char power_num;//0x01 num of power
  buildVariable(power_num, byteStream);
//  std::cout<<"power_num:"<<(unsigned int)power_num<<std::endl;

  build_special_variable(data.battery_voltage,byteStream);
//  std::cout<<"power_voltage:"<<data.power_voltage<<std::endl;
  unsigned char infred_num;//0x03
  buildVariable(infred_num, byteStream);
  buildVariable(data.rear_left_infred,byteStream);
  buildVariable(data.rear_center_infred,byteStream);
  std::cout<<"rear_center_infred:"<<data.rear_center_infred<<std::endl;
  buildVariable(data.rear_right_infred,byteStream);

  unsigned char current_num;//0x05
  buildVariable(current_num,byteStream);
  build_special_variable(data.front_left_current,byteStream);
  build_special_variable(data.front_right_current,byteStream);
  build_special_variable(data.rear_left_current,byteStream);
  build_special_variable(data.rear_right_current,byteStream);
  build_special_variable(data.up_down_current,byteStream);

  unsigned char echo_num;
  buildVariable(echo_num,byteStream);
  build_special_variable(data.front_left_echo,byteStream);
  build_special_variable(data.front_center_echo,byteStream);
  std::cout<<"front_center_echo:"<<data.front_center_echo<<std::endl;
  build_special_variable(data.front_right_echo,byteStream);

  unsigned char encoder_num;
  buildVariable(encoder_num, byteStream);
  buildVariable(data.front_left_encoder, byteStream);
//  std::cout<<"front_left:"<<data.front_left_encoder<<std::endl;
  buildVariable(data.front_right_encoder, byteStream);
//  std::cout<<"front_right:"<<data.front_right_encoder<<std::endl;
  buildVariable(data.rear_left_encoder, byteStream);
  buildVariable(data.rear_right_encoder, byteStream);
  buildVariable(data.up_down_encoder, byteStream);

  unsigned char imu_num;
  buildVariable(imu_num, byteStream);
  buildVariable(data.acce_x, byteStream);
  buildVariable(data.acce_y, byteStream);
  buildVariable(data.acce_z, byteStream);
  buildVariable(data.gyro_x, byteStream);
  buildVariable(data.gyro_y, byteStream);
  buildVariable(data.gyro_z, byteStream);
  buildVariable(data.mag_x, byteStream);
  buildVariable(data.mag_y, byteStream);
  buildVariable(data.mag_z, byteStream);
  buildVariable(data.pressure, byteStream);
  buildVariable(data.yaw,byteStream);
  buildVariable(data.pitch, byteStream);
  buildVariable(data.roll, byteStream);
  buildVariable(data.timestamp, byteStream);

//  std::cout<<"timestamp:"<<data.timestamp<<std::endl;
//  unsigned short x=3;
//  unsigned short y=65500;
//  unsigned short result=x-y;

//  std::cout<<"sizeof timestamp:"<<result<<std::endl;


//  std::cout<<"time:"<<time(0)<<"|left_encoder:"<<data.left_encoder<<std::endl;


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
