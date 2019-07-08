#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import yaml
import roslib
roslib.load_manifest("canusb");
##Mensajes
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from sara_control.msg import enc_msg
from _CAN import CAN
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu

import serial
import struct
import time
import threading
from math import pi

class LECTURA_CLASS:

# Function: __init__
#
# Comments
# ----------
# Constructor of the class LECTURA_CLASS
# Define system constants.
#
# Parameters
# ----------
#
# Returns
# -------

	def __init__(self):

		self.sensor_min_range=0.01 	#m
		self.sensor_max_range= 2.5	#m
		self.field_of_view= pi/3	#rad 60 degree
		self.do_median_calc = True

		self.kimu = 9.49 #Dato/(m/s²) #Constant to convert the data of the imu to m/s²


# Function: start_node
#
# Comments
# ----------
# Initialize the node and the topics.
#
# Parameters
# ----------
#
# Returns
# -------

	def start_node(self):
		rospy.init_node('lectura', anonymous=True)

		self.pubmodo = rospy.Publisher('/modo', Int16, queue_size=100)
		self.pubjoyx = rospy.Publisher('/joyx', Int16, queue_size=100)
		self.pubjoyy = rospy.Publisher('/joyy', Int16, queue_size=100)
		self.pubvelD = rospy.Publisher('/velD', Int16, queue_size=100)
		self.pubvelI = rospy.Publisher('/velI', Int16, queue_size=100)
		self.pubmodoPC = rospy.Publisher('/modoPC', Int16, queue_size=100)
		self.enc_pub = rospy.Publisher("/enc", enc_msg, queue_size=100) #1 topic for both encoders
		self.pubbat = rospy.Publisher('/bat', Int16, queue_size=100)

		self.SLIT_pub = rospy.Publisher("/SLIT_range",Range, queue_size=100)
		self.SLDD_pub = rospy.Publisher("/SLDD_range",Range, queue_size=100)
		self.STD_pub = rospy.Publisher("/STD_range",Range, queue_size=100)
		self.SDI_pub = rospy.Publisher("/SDI_range",Range, queue_size=100)
		self.SLID_pub = rospy.Publisher("/SLID_range",Range, queue_size=100)
		self.SDD_pub = rospy.Publisher("/SDD_range",Range, queue_size=100)
		self.SLDT_pub = rospy.Publisher("/SLDT_range",Range, queue_size=100)
		self.STI_pub = rospy.Publisher("/STI_range",Range, queue_size=100)
		self.SJO_pub = rospy.Publisher('/SJO_range',Range, queue_size=100)

		self.pubImu = rospy.Publisher('/imu', Imu, queue_size=100)
		#self.pubejex = rospy.Publisher('/ejex', Int16, queue_size=100)
		#self.pubejey = rospy.Publisher('/ejey', Int16, queue_size=100)
		#self.pubejez = rospy.Publisher('/ejez', Int16, queue_size=100)

		self.init_sensor_variables()


# Function: init_sensor_variables
#
# Comments
# ----------
# Initialize the constan values of the ultrasonic sensors topics.
#
# Parameters
# ----------
#
# Returns
# -------

	def init_sensor_variables(self):
		#Primer mensaje
		#STD
		self.data_STD=Range()
		self.data_STD.radiation_type=self.data_STD.ULTRASOUND #Ultrasound
		self.data_STD.field_of_view= self.field_of_view #rad
		self.data_STD.min_range=self.sensor_min_range
		self.data_STD.max_range=self.sensor_max_range
		self.data_STD.range=self.sensor_max_range
		self.data_STD.header.stamp = rospy.Time.now()
		self.data_STD.header.frame_id = "STD_frame"

		#SDI
		self.data_SDI=Range()
		self.data_SDI.radiation_type=self.data_SDI.ULTRASOUND #Ultrasound
		self.data_SDI.field_of_view= self.field_of_view #rad
		self.data_SDI.min_range=self.sensor_min_range
		self.data_SDI.max_range=self.sensor_max_range
		self.data_SDI.range= float('Inf')
		self.data_SDI.header.stamp = rospy.Time.now()
		self.data_SDI.header.frame_id = "SDI_frame"

		#SLIT
		self.data_SLIT=Range()
		self.data_SLIT.radiation_type=self.data_SLIT.ULTRASOUND #Ultrasound
		self.data_SLIT.field_of_view= self.field_of_view #rad
		self.data_SLIT.min_range=self.sensor_min_range
		self.data_SLIT.max_range=self.sensor_max_range
		self.data_SLIT.range=self.sensor_max_range
		self.data_SLIT.header.stamp = rospy.Time.now()
		self.data_SLIT.header.frame_id = "SLIT_frame"


		#SLDD
		self.data_SLDD=Range()
		self.data_SLDD.radiation_type=self.data_SLDD.ULTRASOUND #Ultrasound
		self.data_SLDD.field_of_view= self.field_of_view #rad
		self.data_SLDD.min_range=self.sensor_min_range
		self.data_SLDD.max_range=self.sensor_max_range
		self.data_SLDD.range=self.sensor_max_range
		self.data_SLDD.header.stamp = rospy.Time.now()
		self.data_SLDD.header.frame_id = "SLDD_frame"

		#Segundo mensaje
		#SLDT
		self.data_SLDT=Range()
		self.data_SLDT.radiation_type=self.data_SLDT.ULTRASOUND #Ultrasound
		self.data_SLDT.field_of_view= self.field_of_view #rad
		self.data_SLDT.min_range=self.sensor_min_range
		self.data_SLDT.max_range=self.sensor_max_range
		self.data_SLDT.range=self.sensor_max_range

		self.data_SLDT.header.stamp = rospy.Time.now()
		self.data_SLDT.header.frame_id = "SLDT_frame"


		#STI
		self.data_STI=Range()
		self.data_STI.radiation_type=self.data_STI.ULTRASOUND #Ultrasound
		self.data_STI.field_of_view= self.field_of_view #rad
		self.data_STI.min_range=self.sensor_min_range
		self.data_STI.max_range=self.sensor_max_range
		self.data_STI.range=self.sensor_max_range

		self.data_STI.header.stamp = rospy.Time.now()
		self.data_STI.header.frame_id = "STI_frame"


		#SLID
		self.data_SLID=Range()
		self.data_SLID.radiation_type=self.data_SLID.ULTRASOUND #Ultrasound
		self.data_SLID.field_of_view= self.field_of_view #rad
		self.data_SLID.min_range=self.sensor_min_range
		self.data_SLID.max_range=self.sensor_max_range
		self.data_SLID.range=self.sensor_max_range

		self.data_SLID.header.stamp = rospy.Time.now()
		self.data_SLID.header.frame_id = "SLID_frame"


		#SDD
		self.data_SDD=Range()
		self.data_SDD.radiation_type=self.data_SDD.ULTRASOUND #Ultrasound
		self.data_SDD.field_of_view= self.field_of_view #rad
		self.data_SDD.min_range=self.sensor_min_range
		self.data_SDD.max_range=self.sensor_max_range
		self.data_SDD.range=self.sensor_max_range

		self.data_SDD.header.stamp = rospy.Time.now()
		self.data_SDD.header.frame_id = "SDD_frame"


		#Tercer Mensaje
		#SJO
		self.data_SJO=Range()
		self.data_SJO.radiation_type=self.data_SJO.ULTRASOUND #Ultrasound
		self.data_SJO.field_of_view= self.field_of_view #rad
		self.data_SJO.min_range=self.sensor_min_range
		self.data_SJO.max_range=self.sensor_max_range
		self.data_SJO.range=1.0

		self.data_SJO.header.stamp = rospy.Time.now()
		self.data_SJO.header.frame_id = "SJO_frame"

		#Arrays to calculate the median of the ultrasonic data
		self.max_array_ultrasonic=3
		#First message
		self.STD_array=[0]*self.max_array_ultrasonic
		self.SDI_array=[0]*self.max_array_ultrasonic
		self.SLIT_array=[0]*self.max_array_ultrasonic
		self.SLDD_array=[0]*self.max_array_ultrasonic
		self.i=0
		#Second message
		self.SLDT_array=[0]*self.max_array_ultrasonic
		self.STI_array=[0]*self.max_array_ultrasonic
		self.SLID_array=[0]*self.max_array_ultrasonic
		self.SDD_array=[0]*self.max_array_ultrasonic
		self.j=0
		#Third message
		self.SJO_array=[0]*self.max_array_ultrasonic
		self.k=0


# Function: callback
#
# Comments
# ----------
# Callback of the canrx topic.
# In the reception of the ultrasonic sensor topics, is possible to calculate
# the median of the data received setting the flag "do_median_calc" to true.
#
# Parameters
# ----------
#
# Returns
# -------

	def callback(self, msg):

		if msg.stdId == 272:
			#Joystick
			(modo,) = struct.unpack('H',msg.data[4:6])
			(joyx,) = struct.unpack('H',msg.data[2:4])
			(joyy,) = struct.unpack('H',msg.data[6:8])
			(velD,) = struct.unpack('B',msg.data[0:1])
			(velI,) = struct.unpack('B',msg.data[1:2])

			self.pubmodo.publish(int(modo))
			self.pubjoyx.publish(int(joyx))
			self.pubjoyy.publish(int(joyy))
			self.pubvelD.publish(int(velD))
			self.pubvelI.publish(int(velI))

		elif msg.stdId == 273:
			(modoPC,) = struct.unpack('B',msg.data[0:1])

			self.pubmodoPC.publish(int(modoPC))

		elif msg.stdId == 257:
			#Encoder A Right
			(encA,) = struct.unpack('I',msg.data[0:4])
			(tencA,) = struct.unpack('I',msg.data[4:8])

			var_enc = enc_msg()
			var_enc.time = int(tencA)
			var_enc.data = int(encA)
			var_enc.encID=0

			self.enc_pub.publish(var_enc)

		elif msg.stdId == 258:
			#Encoder B Left
			(encB,) = struct.unpack('I',msg.data[0:4])
			(tencB,) = struct.unpack('I',msg.data[4:8])

			var_enc = enc_msg()
			var_enc.time = int(tencB)
			var_enc.data = int(encB)
			var_enc.encID=1

			self.enc_pub.publish(var_enc)

		elif msg.stdId == 528:
			#Battery volt
			(bat,) = struct.unpack('I',msg.data[0:4])

			self.pubbat.publish(int(bat))

		elif msg.stdId == 513:
			#rospy.loginfo("Mensaje sensores 2")
			(STD,) = struct.unpack('H', msg.data[:2])
			(SDI,) = struct.unpack('H', msg.data[2:4])
			(SLIT,) = struct.unpack('H', msg.data[4:6])
			(SLDD,) = struct.unpack('H', msg.data[6:8])

			if self.do_median_calc == True:
				#rospy.loginfo("Tamaño i " + str(self.i))
				self.STD_array[self.i]=STD
				self.SDI_array[self.i]=SDI
				self.SLIT_array[self.i]=SLIT
				self.SLDD_array[self.i]=SLDD
				self.i=(self.i+1)%self.max_array_ultrasonic #3Values


				STD_median=self.calc_median(self.STD_array)
				SDI_median=self.calc_median(self.SDI_array)
				SLIT_median=self.calc_median(self.SLIT_array)
				SLDD_median=self.calc_median(self.SLDD_array)

				self.data_STD.range=STD_median*0.01 #cm to m
				self.data_SDI.range=SDI_median*0.01 #cm to m
				self.data_SLIT.range=SLIT_median*0.01 #cm to m
				self.data_SLDD.range=SLDD_median*0.01 #cm to m

			else:
				self.data_STD.range=STD*0.01 #cm to m
				self.data_SDI.range=SDI*0.01 #cm to m
				self.data_SLIT.range=SLIT*0.01 #cm to m
				self.data_SLDD.range=SLDD*0.01 #cm to m

			#Adjust min and max values
			self.data_STD.range=self.saturate_ultrasonic_data(self.data_STD.range)
			self.data_SDI.range=self.saturate_ultrasonic_data(self.data_SDI.range)
			self.data_SLIT.range=self.saturate_ultrasonic_data(self.data_SLIT.range)
			self.data_SLDD.range=self.saturate_ultrasonic_data(self.data_SLDD.range)

			#STD
			self.data_STD.header.stamp = rospy.Time.now()
			self.STD_pub.publish(self.data_STD)

			#SDI
			self.data_SDI.header.stamp = rospy.Time.now()
			self.SDI_pub.publish(self.data_SDI)

			#SLIT
			self.data_SLIT.header.stamp = rospy.Time.now()
			self.SLIT_pub.publish(self.data_SLIT)

			#SLDD
			self.data_SLDD.header.stamp = rospy.Time.now()
			self.SLDD_pub.publish(self.data_SLDD)

		elif msg.stdId == 514:

			(SLDT,) = struct.unpack('H', msg.data[6:8])
			(STI,) = struct.unpack('H', msg.data[4:6])
			(SLID,) = struct.unpack('H', msg.data[2:4])
			(SDD,) = struct.unpack('H', msg.data[:2])

			if self.do_median_calc == True:
				self.SLDT_array[self.j]=SLDT
				self.STI_array[self.j]=STI
				self.SLID_array[self.j]=SLID
				self.SDD_array[self.j]=SDD
				self.j=(self.j+1)%self.max_array_ultrasonic #3Values

				SLDT_median=self.calc_median(self.SLDT_array)
				STI_median=self.calc_median(self.STI_array)
				SLID_median=self.calc_median(self.SLID_array)
				SDD_median=self.calc_median(self.SDD_array)

				self.data_SLDT.range=SLDT_median*0.01 #cm to m
				self.data_STI.range=STI_median*0.01 #cm to m
				self.data_SLID.range=SLID_median*0.01 #cm to m
				self.data_SDD.range=SDD_median*0.01 #cm to m

			else:
				self.data_SLDT.range=SLDT*0.01 #cm to m
				self.data_STI.range=STI*0.01 #cm to m
				self.data_SLID.range=SLID*0.01 #cm to m
				self.data_SDD.range=SDD*0.01 #cm to m

			#Adjust min and max values
			self.data_SLDT.range=self.saturate_ultrasonic_data(self.data_SLDT.range)
			self.data_STI.range=self.saturate_ultrasonic_data(self.data_STI.range)
			self.data_SLID.range=self.saturate_ultrasonic_data(self.data_SLID.range)
			self.data_SDD.range=self.saturate_ultrasonic_data(self.data_SDD.range)

			#SLDT
			self.data_SLDT.header.stamp = rospy.Time.now()
			self.SLDT_pub.publish(self.data_SLDT)

			#STI
			self.data_STI.header.stamp = rospy.Time.now()
			self.STI_pub.publish(self.data_STI)

			#SLID
			self.data_SLID.header.stamp = rospy.Time.now()
			self.SLID_pub.publish(self.data_SLID)

			#SDD
			self.data_SDD.header.stamp = rospy.Time.now()
			self.SDD_pub.publish(self.data_SDD)


		elif msg.stdId == 515:

			(ejex,) = struct.unpack('H', msg.data[6:8])
			(ejey,) = struct.unpack('H', msg.data[4:6])
			(ejez,) = struct.unpack('H', msg.data[2:4])
			(SJO,) = struct.unpack('H', msg.data[:2])

			#imu
			data_imu=Imu()
			data_imu.header.stamp = rospy.Time.now()

			data_imu.linear_acceleration.x=ejex / self.kimu #[m/s^2]
			data_imu.linear_acceleration.y=ejey / self.kimu #[m/s^2]
			data_imu.linear_acceleration.y=ejez / self.kimu #[m/s^2]
			#If you have no estimate for one of the data elements  please set element 0 of the associated covariance matrix to -1
			data_imu.linear_acceleration_covariance[0] = -1

			self.pubImu.publish(data_imu)

			#Joystick ultrasonic sensor

			if self.do_median_calc == True:
				self.SJO_array[self.k]=SJO
				self.k=(self.k+1)%self.max_array_ultrasonic #3Values

				SJO_median=self.calc_median(self.SJO_array)

				self.data_SJO.range=SJO_median*0.01 #cm to m


			else:
				self.data_SJO.range=SJO*0.01 #cm to m


			#Adjust min and max values
			self.data_SJO.range=self.saturate_ultrasonic_data(self.data_SJO.range)

			self.data_SJO.header.stamp = rospy.Time.now()
			self.SJO_pub.publish(self.data_SJO)




# Function: calc_median
#
# Comments
# ----------
# Calculate the median of an array
#
# Parameters
# ----------
# array: Array to calculate the median
#
# Returns
# -------
# mediana: Median value of the array
	def	calc_median(self,array):

		dOrder=sorted(array)
		middle=int(self.max_array_ultrasonic/2)
		mediana=dOrder[middle+1]

		return mediana

# Function: saturate_ultrasonic_data
#
# Comments
# ----------
# Limit the max and min reading of the ultrasonics sensor.
# It allows to clean the obstacles in range_sensor_layer
#
# Parameters
# ----------
#
# Returns
# -------
# Corrected value
	def	saturate_ultrasonic_data(self,data):

		if data == 0: #Infinite value
			return float('Inf')
		elif data > self.sensor_max_range:
			return self.sensor_max_range
		elif data<self.sensor_min_range:
			return self.sensor_min_range
		else: #Inside the limits of range
			return data


# Function: debug_publish
#
# Comments
# ----------
# Pubic a fixed value of the range of the ultrasonic sesors.
# Used for debug
#
# Parameters
# ----------
#
# Returns
# -------
	def debug_publish(self):
		r = rospy.Rate(5.0)

		while not rospy.is_shutdown():
			self.data_STD.header.stamp = rospy.Time.now()
			self.data_SDI.header.stamp = rospy.Time.now()
			self.data_SLIT.header.stamp = rospy.Time.now()
			self.data_SLDD.header.stamp = rospy.Time.now()

			self.data_SLDT.header.stamp = rospy.Time.now()
			self.data_STI.header.stamp = rospy.Time.now()
			self.data_SLID.header.stamp = rospy.Time.now()
			self.data_SDD.header.stamp = rospy.Time.now()

			self.data_SJO.header.stamp = rospy.Time.now()

			self.STD_pub.publish(self.data_STD)
			self.SDI_pub.publish(self.data_SDI)
			self.SLIT_pub.publish(self.data_SLIT)
			self.SLDD_pub.publish(self.data_SLDD)

			self.SLDT_pub.publish(self.data_SLDT)
			self.STI_pub.publish(self.data_STI)
			self.SLID_pub.publish(self.data_SLID)
			self.SDD_pub.publish(self.data_SDD)

			self.SJO_pub.publish(self.data_SJO)

			r.sleep()


# Function: lisener
#
# Comments
# ----------
# Function who start the subsciption to the "canrx" topic and wait for messages
#
# Parameters
# ----------
#
# Returns
# -------

	def lisener(self):
		rospy.Subscriber("canrx", CAN, self.callback)
		rospy.loginfo("Lectura CAN iniciado")
		rospy.spin()


if __name__ == '__main__':

	lectura=LECTURA_CLASS()
	lectura.start_node()
	try:
		lectura.lisener()
		#lectura.debug_publish()
	except rospy.ROSInterruptException: pass
