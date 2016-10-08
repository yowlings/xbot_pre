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


def callback(data):
    print data.power
    
def xbot_state_subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('xbot_state_subscriber', anonymous=True)

    rospy.Subscriber("xbot_state", xbot_state, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    xbot_state_subscriber()