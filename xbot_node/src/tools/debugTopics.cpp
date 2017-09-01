#include "ros/ros.h"
#include "xbot_msgs/Power.h"
#include "xbot_msgs/CloudCamera.h"
#include "xbot_msgs/Lift.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debugTopics");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<xbot_msgs::Power>("/mobile_base/commands/power", 1000);

  ros::Rate loop_rate(10);
  while(ros::ok()){
    xbot_msgs::Power msg;
    msg.header.stamp = ros::Time::now();
    msg.power = xbot_msgs::Power::OFF;

    chatter_pub.publish(msg);

    ros::spinOnce();

  }




  return 0;
}
