#include <queue>
#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H


#define WHEEL_SPEED 0x09
#define CRADLE_HEIGHT 0x03
#define POWER_SWITH 0x02
#define CAMERA_ANGLE 0x03

enum Action{POWER=1,SPEED=3,HEIGHT=4,ANGLE=5};

class Frame
{
private:
    unsigned char header[2];//帧头
    unsigned char data_len;//数据域长度
    unsigned char data[20];//数据域
    int frame_len;//帧长
    unsigned char frame[30];//帧
    float v_translate, v_rotate;//速度
    bool power;//电源开关
    unsigned char height;//云台高度(percent)   大于100代表停止
    unsigned char pitch;//摄像机俯仰角(angle)
    unsigned char yaw;//云台偏航角(angle)
////////////////////////////////
    short int acce_x;
    short int acce_y;
    short int acce_z;
    short int gyro_x;
    short int gyro_y;
    short int gyro_z;
    short int mag_x;
    short int mag_y;
    short int mag_z;
    float pressure;
    short int ang_yaw;
    short int ang_pitch;
    short int ang_roll;
    unsigned int timestamp;
    unsigned short int motor1_encoder;
    unsigned short int motor2_encoder;
    unsigned short int motor3_encoder;
    float l_speed,r_speed; //left and right wheel speed acquired from encoder
public:
    Frame() ;
    void set_power(bool p);
    void set_speed(float v_t,float v_r);
    void set_height(unsigned char h);
    void set_yaw(unsigned char yaw);
    void set_pitch(unsigned char pitch);
    float get_v_t(){return v_translate;}
    float get_v_r(){return v_rotate;}
    unsigned char* update_data(int action);//得到数据域(data)的内容
    unsigned char update_frame_end();//得到帧尾
    unsigned char* update_frame(Action action);//更新帧
    unsigned char* get_frame();//得到帧
    int get_frame_len();////得到帧长
////////////////////////////////////////////
    short int get_acce_x() {return acce_x;}
    short int get_acce_y() {return acce_y;}
    short int get_acce_z() {return acce_z;}
    short int get_gyro_x() {return gyro_x;}
    short int get_gyro_y() {return gyro_y;}
    short int get_gyro_z() {return gyro_z;}
    short int get_mag_x() {return mag_x;}
    short int get_mag_y() {return mag_y;}
    short int get_mag_z() {return mag_z;}
    float get_pressure() {return pressure;}
    short int get_ang_yaw() {return ang_yaw;}
    short int get_ang_pitch() {return ang_pitch;}
    short int get_ang_roll() {return ang_roll;}
    unsigned int get_timestamp() {return timestamp;}
    unsigned int last_timestamp;
    unsigned int delta_timestamp;
    unsigned short last_motor1_encoder;
    unsigned short last_motor2_encoder;
    unsigned short last_motor3_encoder;
    unsigned short  get_motor1_encoder() {return motor1_encoder;}
    unsigned short  get_motor2_encoder() {return motor2_encoder;}
    unsigned short  get_motor3_encoder() {return motor3_encoder;}
    void update_wheel_speed();
    float get_lspeed() {return l_speed;}
    float get_rspeed() {return r_speed;}
    unsigned int get_delta_timestamp() {return delta_timestamp;}

    std::queue<unsigned char> rbuff_q;
    int update_sensor_data();
};






#endif // ROBOT_FRAME_H
