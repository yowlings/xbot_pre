#!/usr/bin/env python
#coding=utf-8
# import xbot_driver
import numpy as np
import thread
import serial
import time
import binascii
import xbot_driver
import sys
myxbot = xbot_driver.xbot_driver()

thread.start_new_thread(myxbot.default_cmd,())

# thread.start_new_thread(myxbot.move,(0.3,0.4))
# thread.start_new_thread(myxbot.lift,(10,))
# myxbot.lift(30)

# myxbot.turn_platform_camera(90,90)
# time.sleep(0.1)
# myxbot.lift(10)

while 1:
	myxbot.move(0.1,20)
	

