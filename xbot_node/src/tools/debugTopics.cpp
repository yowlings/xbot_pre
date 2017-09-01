#include "ros/ros.h"
#include "xbot_msgs/Power.h"
#include "xbot_msgs/CloudCamera.h"
#include "xbot_msgs/Lift.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debugTopics");
  ros::NodeHandle nh;
// ################## debug power enable ################
//  ros::Publisher chatter_pub = nh.advertise<xbot_msgs::Power>("/mobile_base/commands/power", 10);
//  ros::Rate loop_rate(10);
//  for(int i=0;i<4;i++){
//    xbot_msgs::Power msg;
//    msg.header.stamp = ros::Time::now();
//    msg.power = xbot_msgs::Power::ON;
//    chatter_pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();

//  }
// ################## debug cloud camera ################
  ros::Publisher chatter_pub = nh.advertise<xbot_msgs::CloudCamera>("/mobile_base/commands/cloud_camera", 10);
  ros::Rate loop_rate(10);
  for(int i=0;i<4;i++){
    xbot_msgs::CloudCamera msg;
    msg.header.stamp = ros::Time::now();
    msg.cloud_degree = 60;
    msg.camera_degree = 135;
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }




  return 0;
}
