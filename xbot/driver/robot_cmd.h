#include "robot_frame.h"
#include <unistd.h>
#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#define Frequency 50
#include <stdio.h>
class Xbot
{
public:
    void lock();
    void release();
    void lift(int percent);
    void lift_stop();
    void turn_platform_camera(int platform_angular,int camera_angular);
    void move(float linear_speed,float angular_speed);
    void slow_down(int msec);
    void slow_up(int msec);

    short  get_acc_x() {return myframe.get_acce_x();}
    short  get_acc_y() {return myframe.get_acce_y();}
    short  get_acc_z() {return myframe.get_acce_z();}
    short  get_gyr_x() {return myframe.get_gyro_x();}
    short  get_gyr_y() {return myframe.get_gyro_y();}
    short  get_gyr_z() {return myframe.get_gyro_z();}
    short  get_mag_x() {return myframe.get_mag_x();}
    short  get_mag_y() {return myframe.get_mag_y();}
    short  get_mag_z() {return myframe.get_mag_z();}
    float get_pres() {return myframe.get_pressure();}
    float  get_yaw() {return 0.1*myframe.get_ang_yaw();}
    float  get_pitch() {return 0.01*myframe.get_ang_pitch();}
    float  get_roll() {return 0.01*myframe.get_ang_roll();}
    unsigned  get_stamp() {return myframe.get_timestamp();}
    unsigned short  get_encoder1() {return myframe.get_motor1_encoder();}
    unsigned short  get_encoder2() {return myframe.get_motor2_encoder();}
    unsigned short  get_encoder3() {return myframe.get_motor3_encoder();}
    float get_left_speed() {return myframe.get_lspeed();}
    float get_right_speed() {return myframe.get_rspeed();}
    Frame& get_frame();
private:
    Frame myframe;
};



#endif // ROBOT_CMD_H
