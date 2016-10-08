#include "robot_cmd.h"
Frame& Xbot::get_frame()
{
    return myframe;
}
void Xbot::lock()
{
    myframe.set_power(false);
    myframe.update_frame(POWER);
}
void Xbot::release()
{
    myframe.set_power(true);
    myframe.update_frame(POWER);
}
void Xbot::lift(int percent)
{
    if(percent<=100&&percent>=0)
    myframe.set_height((unsigned char)percent);
    myframe.update_frame(HEIGHT);
}
void Xbot::lift_stop()
{
    myframe.set_height(0xff);
    myframe.update_frame(HEIGHT);
}
void Xbot::turn_platform_camera(int platform_angular,int camera_angular)
{
    myframe.set_yaw((unsigned char)platform_angular);
    myframe.set_pitch((unsigned char)camera_angular);
    myframe.update_frame(ANGLE);
}
void Xbot::move(float linear_speed,float angular_speed)
{
     myframe.set_speed(linear_speed,angular_speed);
     myframe.update_frame(SPEED);
}
void Xbot::slow_down(int msec)
{
    float v_t=myframe.get_v_t();
    float v_r=myframe.get_v_r();
    float dv_t= v_t/(msec*Frequency/1000);//速度以Frequency的频率 经过msec 下降至0
    float dv_r=v_r/(msec*Frequency/1000);//计算得到差分量
    for(int i=msec;i>0;i-=1000/Frequency)
    {
        v_t-=dv_t;
        v_r-=dv_r;
        myframe.set_speed(v_t,v_r);
        myframe.update_frame(SPEED);
        printf("v_t=%f,   v_r=%f\n",v_t,v_r);
        usleep(1000000/Frequency);//20ms
    }
    myframe.set_speed(0,0);
}

