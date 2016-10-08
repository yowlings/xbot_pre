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
import thread
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

from xbot_bringup.msg import xbot_cmd
import xbot_driver

class xbot_ros(object):
	"""docstring for xbot_ros"""



	def __init__(self):
		self.clear = True
		self.myxbot = xbot_driver.xbot_driver()
		thread.start_new_thread(self.myxbot.default_cmd,())
		rospy.init_node('xbot_controller')
		
		
		rospy.Subscriber('xbot_control', xbot_cmd, self.control_callback)
		rospy.spin()






	def control_callback(self,data):
		print 'received'
		control_type = data.cmd
		control_args = data.args
		if control_type == 'move':
			try:
				self.myxbot.move(control_args[0],control_args[1])
			except Exception, e:
				print 'move failed'
			
		elif control_type == 'lift':
			try:
				self.myxbot.lift(control_args[0])
			except Exception, e:
				print 'lift failed'

		elif control_type == 'turn_platform_camera':
			try:
				self.myxbot.turn_platform_camera(control_args[0],control_args[1])
			except Exception, e:
				print 'turn_platform_camera failed'

		else:
			print 'control command fault'







if __name__=='__main__':
	try:
		rospy.loginfo ("initialization system")
		xbot_ros()
		rospy.loginfo ("process done and quit")
	except rospy.ROSInterruptException:
		rospy.loginfo("robot twist node terminated.")
		