#!/usr/bin/env python
# coding=utf-8
"""
xbot_safety.py
"""

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from collections import deque


class xbot_scan180():
    """
    reduce rplidar scan from 360° to 180°
    """

    def __init__(self):
        rospy.init_node('xbot_scan180')
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        rospy.Subscriber("/rplidar_scan", LaserScan, self.scan_dataCB)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()


    def scan_dataCB(self, scan_data):
        scan_data.ranges = list(scan_data.ranges)
        for i in xrange(len(scan_data.ranges)):
            if i < 90 or i >= 270:
                scan_data.ranges[i] = float('inf')
        scan_data.ranges = tuple(scan_data.ranges)
        self.pub.publish(scan_data)


if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_scan180()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_scan180 terminated.")


