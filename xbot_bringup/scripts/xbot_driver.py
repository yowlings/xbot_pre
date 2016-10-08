#!/usr/bin/env python
#coding=utf-8
import numpy as np
import thread
import serial
import time
import binascii
from struct import pack, unpack

class xbot_driver(object):


	def __init__(self):
		self.sync_code = "\xAA\x55"
		self.power_protocol = "\x02\x01"
		self.move_protocol = "\x09\x03"
		self.lift_protocol = "\x03\x04\x01"
		self.platform_camera_protocol = "\x03\x05"
		self.cmd = self.sync_code + self.lift_protocol + '\x00\x06'
		try:
			self.ser = serial.Serial('/dev/xbot', 115200, timeout=1)
			
		except Exception, e:
			print "please plug in the xbot"
		else:
			return
		finally:
			pass


	def default_cmd(self):
		while 1:			
			self.ser.write(self.cmd)
			# print 'yes'
			time.sleep(0.02)
		
		



	def lock(self):
		data = self.sync_code + self.power_protocol +"\x00\x03"
		for i in range(50):
			self.ser.write(data)
			time.sleep(0.02)


	def release(self):

		data = self.sync_code + self.power_protocol +"\x01\x02"
		for i in range(50):
			self.ser.write(data)
			time.sleep(0.02)
		
	def lift(self, percent):

		percent = implement(percent)
		percent_decode = percent.decode("hex")
		data = self.sync_code + self.lift_protocol + percent_decode
		crc = get_crc(data)
		data = data + crc
		self.cmd = data

	def turn_platform_camera(self,platform_angle,camera_angle):

		platform_angle = implement(platform_angle)
		camera_angle = implement(camera_angle)
		platform_angle_decode = platform_angle.decode("hex")
		camera_angle_decode = camera_angle.decode("hex")
		data = self.sync_code + self.platform_camera_protocol + platform_angle_decode + camera_angle_decode
		crc = get_crc(data)
		data = data +crc
		self.cmd = data
		# self.ser.write(data)

	def move(self,linear_speed,angular_speed):
		linear_speed = pack('f',linear_speed)
		angular_speed = pack('f',angular_speed)
		data = self.sync_code + self.move_protocol + linear_speed + angular_speed
		crc = get_crc(data)
		data = data + crc
		self.cmd = data
		# for i in range(500):
		# 	self.ser.write(data)
		# 	time.sleep(0.02)



	def get_data(self,size):
		return self.ser.read(size)


	def slow_down(self):
		pass



	def process_data(self,size):
		data = self.get_data(size)
		index = []
		for i in range(0,len(data)-1):
			if (data[i]=='\xaa')&(data[i+1]=='\x55'):
				index.append(i)
		power_state = []
		IMU_state = []
		# print len(index)

		for i in range(0,len(index)-1):
			data_piece = data[index[i]:index[i+1]]
			data_piece_data = data_piece[0:len(data_piece)-1]
			crc = get_crc(data_piece_data)
			if crc == data_piece[-1]:
				if len(data_piece) == 40:
					power_state.append(data_piece)
				elif len(data_piece) == 37:
					IMU_state.append(data_piece)
	
		# print len(IMU_state)
		# print len(power_state)
		for imu in IMU_state:
			acce_x = unpack('H',imu[4:6])[0]
			acce_y = unpack('H',imu[6:8])[0]
			acce_z = unpack('H',imu[8:10])[0]
			gyro_x = unpack('H',imu[10:12])[0]
			gyro_y = unpack('H',imu[12:14])[0]
			gyro_z = unpack('H',imu[14:16])[0]
			mag_x = unpack('H',imu[16:18])[0]
			mag_y = unpack('H',imu[18:20])[0]
			mag_z = unpack('H',imu[20:22])[0]
			# print acce_x,acce_y,acce_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
			pressure = unpack('f',imu[22:26])
			# print pressure
			yaw = unpack('H',imu[26:28])[0]
			pitch = unpack('H',imu[28:30])[0]
			roll = unpack('H',imu[30:32])[0]
			# print yaw,pitch,roll
			timestamp = unpack('I',imu[32:36])[0]
			# print timestamp

		for power in power_state:
			power_v = hexdirect2float(power[5:7])
			current_1 = hexdirect2float(power[9:11])
			current_2 = hexdirect2float(power[11:13])
			current_3 = hexdirect2float(power[13:15])
			current_4 = hexdirect2float(power[15:17])
			current_5 = hexdirect2float(power[17:19])
			echo_1 = hexdirect2float(power[20:22])
			echo_2 = hexdirect2float(power[22:24])
			echo_3 = hexdirect2float(power[24:26])
			echo_4 = hexdirect2float(power[26:28])
			motor_1 = unpack('H',power[29:31])[0]
			motor_2 = unpack('H',power[31:33])[0]
			motor_3 = unpack('H',power[33:35])[0]
			motor_4 = unpack('H',power[35:37])[0]
			motor_5 = unpack('H',power[37:39])[0]
		# return {'power_v':power_v,'current_1':current_1,'current_2':current_2,\
		# 	'current_3':current_3,'current_4':current_4,'current_5':current_5,\
		# 	'echo_1':echo_1,'echo_2':echo_2,'echo_3':echo_3,'echo_4':echo_4,\
		# 	'motor_1':motor_1,'motor_2':motor_2,'motor_3':motor_3,'motor_4':motor_4,\
		# 	'motor_5':motor_5,'acce_x':acce_x,'acce_y':acce_y,'acce_z':acce_z,\
		# 	'gyro_x':gyro_x,'gyro_y':gyro_y,'gyro_z':gyro_z,'mag_x':mag_x,'mag_y':mag_y,\
		# 	'mag_z':mag_z,'pressure':pressure,'yaw':yaw,'pitch':pitch,'roll':roll,\
		# 	'timestamp':timestamp}
			return power_v



def implement(data):

	if data < 16:
		return "0%x"%data
	else:
		return "%x"%data

def get_crc(data):
	
	a = data
	a = a.encode("hex")
	b = binascii.unhexlify(a)
	c = [ord(x) for x in b]
	crc=c[2]
	for i in range(2,len(c)-1):
		crc=crc^c[i+1]

	crc = implement(crc)

	crc = crc.decode("hex")
	return crc

def hexdirect2float(data):
	b=data.encode("hex")
	c=b[2:4]
	d=b[0:2]
	e=int(c)+int(d)/100.0
	return e
