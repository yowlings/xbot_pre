#!/usr/bin/env python
#coding=utf-8
""" 

"""
# Python includes
import numpy
import random

# ROS includes
import roslib
import rospy
import xbot_driver
from xbot_bringup.msg import xbot_state

def xbot_state_publisher():
    pub = rospy.Publisher('xbot_state', xbot_state, queue_size=1000)
    rospy.init_node('xbot_state_publisher', anonymous=True)
    myxbot = xbot_driver.xbot_driver()
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
    	xbot_power = myxbot.process_data(500)
        state = xbot_state()
        state.power = xbot_power
       	pub.publish(state)
        rate.sleep()

   
if __name__ == '__main__':
    try:
        xbot_state_publisher()
    except rospy.ROSInterruptException:
        pass