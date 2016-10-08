#include "robot_frame.h"
#include <string.h>
#include <stdio.h>
Frame::Frame()
{
    power=false;
    height=0;
    pitch=90;
    yaw=90;
    header[0]=0xaa;
    header[1]=0x55;
    v_translate=0.0f;
    v_rotate=0.0f;
    data_len=WHEEL_SPEED;//数据域长度


    delta_timestamp=20000;
}
void Frame::set_power(bool p)
{
    power=p;
}
void Frame::set_speed(float v_t,float v_r)
{
    v_translate=v_t;
    v_rotate=v_r;
}
void Frame::set_height(unsigned char h)
{
    height=h;
}
void Frame::set_yaw(unsigned char y)
{
    yaw=y;
}
void Frame::set_pitch(unsigned char p)
{
    pitch=p;
}
unsigned char* Frame::update_data(int action)
{
    switch(action)
    {
        case SPEED:
            data_len=WHEEL_SPEED;
            data[0]=0x03;//数据域的标识符
            memcpy(data+1,&v_translate,4);//小端模式  memcpy
            memcpy(data+5,&v_rotate,4);
            break;
        case POWER:
            data_len=POWER_SWITH;
            data[0]=0x01;
            if(power==true)
                data[1]=0x01;
            else
                data[1]=0x00;
            break;
        case HEIGHT:
            data_len=CRADLE_HEIGHT;
            data[0]=0x04;
            if(height>100)
            {
                data[1]=0x00;
                data[2]=0x00;
            }
            else
            {
                data[1]=0x01;
                data[2]=height;
            }
        break;
        case ANGLE:
            data_len=CAMERA_ANGLE;
            data[0]=0x05;
            data[1]=yaw;
            data[2]=pitch;
        break;
    }
    return data;
    //if(....)

/*        printf("data is ");
    for(int i=0;i<data_len;i++)
    {
        printf("%02x ",data[i]);
    }
    printf("\n");
*/
}



unsigned char Frame::update_frame_end()
{
    //计算得到帧尾
    unsigned char frame_end=data_len;
    for(unsigned char i=0;i<data_len;i++)
    {
        frame_end^=data[i]; //除了帧头外所有数据的异或
    }
    return frame_end;
}

unsigned char* Frame::get_frame()
{
    return frame;
}

int Frame::get_frame_len()
{
    return frame_len;
}
unsigned char* Frame::update_frame(Action action)
{
    update_data(action);
    frame_len=2+1+(int)data_len+1;
    memcpy(frame,&header,2);
    memcpy(frame+2,&data_len,1);
    memcpy(frame+3,data,data_len);
    frame[frame_len-1]=update_frame_end();

//    for(int i=0;i<frame_len;i++)
//    {
//        printf("%02x ",frame[i]);
//    }
//    printf("\n");
    return frame;
}

void Frame::update_wheel_speed()
{
    if(timestamp!=last_timestamp)
       delta_timestamp=timestamp-last_timestamp;
    unsigned short d_left_encoder=motor1_encoder-last_motor1_encoder;
    unsigned short d_right_encoder=motor2_encoder-last_motor2_encoder;
//    printf("delta_left_encoder:%d     delta_right_encoder:%d\n",d_left_encoder,d_right_encoder);
    printf("delta_time:%d\n",delta_timestamp);
    if(d_left_encoder>30000) //rotate reversely
    {
        d_left_encoder=65535-d_left_encoder;
        l_speed=-1000.0*d_left_encoder/(delta_timestamp)/15.06; //15.06 is a K
    }
    else
    {
        l_speed=1000.0*d_left_encoder/(delta_timestamp)/15.06; //15.06 is a K
    }
    if(d_right_encoder>30000)//rotate reversely
    {
        d_right_encoder=65535-d_right_encoder;
        r_speed=-1000.0*d_right_encoder/(delta_timestamp)/15.06;
    }
    else
    {
        r_speed=1000.0*d_right_encoder/(delta_timestamp)/15.06;
    }
    printf("l_speed:%f   r_speed:%f\n",l_speed,r_speed);
}

int Frame::update_sensor_data()
{
    int popnum=0;
    while(rbuff_q.front()!=0xaa&&rbuff_q.size()>0)
        {
           // printf("%02x",rbuff_q.front());
            rbuff_q.pop();
            popnum++;
        }
    if(popnum>0) printf("%d elems pop\n",popnum);
    popnum=0;

    while(rbuff_q.size()>=37)  //start with 0x55 and length is greater than 37,which must be a complete frame
    {
        rbuff_q.pop();//pop 0xaa
        if(rbuff_q.front()==0x55)
        {
            rbuff_q.pop();//pop 0x55
            int len=rbuff_q.front();//get data length
            rbuff_q.pop();
           if(rbuff_q.front()==0x11&&len==33)
            {
                unsigned char IMUdata[len];
                for(int i=0;i<len;i++)
                {
                    IMUdata[i]=rbuff_q.front();
   //                 printf("%02x",IMUdata[i]);
                    rbuff_q.pop();
                }
                rbuff_q.pop();//pop crc
//                printf("\n");
                acce_x=*((short *)&IMUdata[1]);
                acce_y=*((short *)&IMUdata[3]);
                acce_z=*((short *)&IMUdata[5]);
                gyro_x=*((short *)&IMUdata[7]);
                gyro_y=*((short *)&IMUdata[9]);
                gyro_z=*((short *)&IMUdata[11]);
                mag_x=*((short *)&IMUdata[13]);
                mag_y=*((short *)&IMUdata[15]);
                mag_z=*((short *)&IMUdata[17]);
                pressure=*((float *)&IMUdata[19]);
                ang_yaw=*((short *)&IMUdata[23]);
                ang_pitch=*((short *)&IMUdata[25]);
                ang_roll=*((short *)&IMUdata[27]);
                last_timestamp=timestamp;
                timestamp=*((unsigned int*)&IMUdata[29]);
            }
            else if(rbuff_q.front()==0x10&&len==32)
            {
                unsigned char Sensordata[len];
                for(int i=0;i<len;i++)
                {
                    Sensordata[i]=rbuff_q.front();
                    rbuff_q.pop();
    //                printf("%02x",Sensordata[i]);
                }
                rbuff_q.pop(); //pop crc
   //             printf("\n");
                //other.......
                //sensor.......
                //messages.......
                last_motor1_encoder=motor1_encoder;
                last_motor2_encoder=motor2_encoder;
                last_motor3_encoder=motor3_encoder;
                motor1_encoder=*((unsigned short *)&Sensordata[26]);
                motor2_encoder=*((unsigned short *)&Sensordata[28]);
                motor3_encoder=*((unsigned short *)&Sensordata[30]);
               update_wheel_speed();
            }
           else
           {
            printf("invalid data rbuff_q.front()=%02x,  len = %d",rbuff_q.front(),len);
           }
        }
    }
}
