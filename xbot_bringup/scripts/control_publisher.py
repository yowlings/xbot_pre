#!/usr/bin/env python
#coding=utf-8
""" 
ROS driver for xbot
"""
# Python includes
import numpy
import random

# ROS includes
import roslib
import rospy
from xbot_bringup.msg import xbot_cmd



def xbot_control_publisher():
    pub = rospy.Publisher('xbot_control', xbot_cmd, queue_size=10)
    rospy.init_node('xbot_control_publisher', anonymous=True)
    control_data = xbot_cmd()
    control_data.cmd = 'lift'
    control_data.args = (10,0.5)
    pub.publish(control_data)

    rospy.spin()

   
if __name__ == '__main__':
    try:
        xbot_control_publisher()
    except rospy.ROSInterruptException:
        pass