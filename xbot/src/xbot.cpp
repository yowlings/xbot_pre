#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "../driver/robot_cmd.h"
#include "../driver/robot_frame.h"
#include "../driver/robot_serial.h"
#include <sys/time.h>

// message declarations
#define Frequency 50
#define PI 3.141592653
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;  //tf topic
sensor_msgs::JointState joint_state;
geometry_msgs::Twist cmd_vel;
Xbot myxbot;
RSerial myserial;
struct timeval lasttime,nowtime;
float cmd_vel_angular_z=0;

void update_Odometry()
{
  // update transform
  odom_trans.header.stamp = ros::Time::now();
  lasttime=nowtime;
  gettimeofday(&nowtime,0);
  double deltaT=(double)(nowtime.tv_usec-lasttime.tv_usec)/1000000+(nowtime.tv_sec-lasttime.tv_sec);
  ROS_INFO("deltaT: [%lf]", deltaT);

  //odom_trans.transform.translation.x+= 0.07*cmd_vel.linear.x*cos(cmd_vel_angular_z);
  //odom_trans.transform.translation.y+= 0.07*cmd_vel.linear.x*sin(cmd_vel_angular_z);
  //odom_trans.transform.translation.z =0.2;
  //cmd_vel_angular_z+=0.07*cmd_vel.angular.z;
  //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(cmd_vel_angular_z);
  cmd_vel_angular_z+=deltaT*(myxbot.get_right_speed()-myxbot.get_left_speed())/0.4;//w=(v1-v2)/(2R)
  if(cmd_vel_angular_z>PI)
  {
    cmd_vel_angular_z-=2*PI;
  }
  else if(cmd_vel_angular_z<-PI)
  {
    cmd_vel_angular_z+=2*PI;
  }
  odom_trans.transform.translation.x+= deltaT*(myxbot.get_left_speed()+myxbot.get_right_speed())/2*cos(cmd_vel_angular_z);
  odom_trans.transform.translation.y+= deltaT*(myxbot.get_left_speed()+myxbot.get_right_speed())/2*sin(cmd_vel_angular_z);
  odom_trans.transform.translation.z =0.2;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(cmd_vel_angular_z);
  ROS_INFO("cmd_vel_angular_z: [%f]", cmd_vel_angular_z);

  //update odom
  // Header
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id =" odom";
  odom.child_frame_id = "base_link";
  // Position
  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.2;
 // odom.pose.pose.orientation = ;

  // Velocity
  //odom.twist.twist.linear.x = pose_update_rates[0];
  //odom.twist.twist.linear.y = pose_update_rates[1];
  //odom.twist.twist.angular.z = pose_update_rates[2];

}
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr cmdvel)  //cmd_vel
{
  cmd_vel=*cmdvel;
  myxbot.move((float)(cmdvel->linear.x),-180/3.1416*(float)(cmdvel->angular.z));
}
void* rwcmd (void *arg)    //a thread which reads and writes commands
{
    printf("sendcmd thread activated\n");
    while(true)
    {
      myserial.swrite(myxbot.get_frame());  //frame对象   串口发送指令帧
      usleep(1000000/2/Frequency);//10ms
      myserial.sread(myxbot.get_frame());
      usleep(1000000/2/Frequency);//10ms
    }
}
int startthread()
{
  pthread_t thread_a;
  int *thread_ret=NULL;
  int ret = pthread_create( &thread_a, NULL, rwcmd, NULL );
  if(ret!=0)
  {
      printf("Create thread error!\n");
      return -1;
  }
  return 1;
}

//bool tfempty(geometry_msgs::Transform t)
//{
//  if(!t.translation.x)
//    return false;
//  if(!t.translation.y)
//    return false;
//  if(!t.translation.z)
//    return false;
//  if(!t.rotation.x)
//    return false;
//  if(!t.rotation.y)
//    return false;
//  if(!t.rotation.z)
//    return false;
//  if(!t.rotation.w)
//    return false;
//  return true;
//}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "xbotnode");
  ros::NodeHandle n;
  ros::Publisher odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);///
  tf::TransformBroadcaster broadcaster;  ///
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);
    myserial.initial("/dev/xbot");
    if(startthread()<0)  return -1;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x= 0.2;
    odom_trans.transform.translation.y= 0;
    odom_trans.transform.translation.z = 0.2;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id =" odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = odom_trans.transform.translation.x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y;
    odom.pose.pose.position.z = 0.2;
    gettimeofday(&nowtime,0);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
        joint_state.name[0] ="base_to_left_wheel";
        joint_state.position[0] = 0;
        joint_state.name[1] ="base_to_right_wheel";
        joint_state.position[1] = 0;
        joint_state.name[2] ="platform1_to_platform2";
        joint_state.position[2] = 0;
        joint_state.name[3] ="platform2_to_platform3";
        joint_state.position[3] = 0;
        joint_state.name[4] ="platform4_to_platform5";
        joint_state.position[4] = 0;
        //send the joint state and transform
        update_Odometry();   //update odom and tf

        joint_pub.publish(joint_state);   //publish /joint_state

   //     if(tfempty (odom_trans.transform))
   //     {
          broadcaster.sendTransform(odom_trans);   //publish /tf
          odom_publisher.publish(odom);  //publish /odom
     //   }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
