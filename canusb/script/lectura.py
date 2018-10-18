#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Creador: FJR 2018
#Funcion:
#Recibe:
#Envia:
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
		self.sensor_max_range= 3.0	#m
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
		self.enc_pub = rospy.Publisher("enc", enc_msg, queue_size=100) #1 topic for both encoders
		self.pubbat = rospy.Publisher('/bat', Int16, queue_size=100)

		self.SLIT_pub = rospy.Publisher("/SLIT_range",Range, queue_size=100)
		self.SLDD_pub = rospy.Publisher("/SLDD_range",Range, queue_size=100)
		self.STD_pub = rospy.Publisher("/STD_range",Range, queue_size=100)
		self.SDI_pub = rospy.Publisher("/SDI_range",Range, queue_size=100)
		self.SLID_pub = rospy.Publisher("/SLID_range",Range, queue_size=100)
		self.SDD_pub = rospy.Publisher("/SDD_range",Range, queue_size=100)
		self.SLDT_pub = rospy.Publisher("/SLDT_range",Range, queue_size=100)
		self.STI_pub = rospy.Publisher("/STI_range",Range, queue_size=100)

		self.pubSJO = rospy.Publisher('/SJO', Int16, queue_size=100)
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
		self.data_STD.range=4.0
		self.data_STD.header.stamp = rospy.Time.now()
		self.data_STD.header.frame_id = "STD_frame"

		#SDI
		self.data_SDI=Range()
		self.data_SDI.radiation_type=self.data_SDI.ULTRASOUND #Ultrasound
		self.data_SDI.field_of_view= self.field_of_view #rad
		self.data_SDI.min_range=self.sensor_min_range
		self.data_SDI.max_range=self.sensor_max_range
		self.data_SDI.range=1.4
		self.data_SDI.header.stamp = rospy.Time.now()
		self.data_SDI.header.frame_id = "SDI_frame"

		#SLIT
		self.data_SLIT=Range()
		self.data_SLIT.radiation_type=self.data_SLIT.ULTRASOUND #Ultrasound
		self.data_SLIT.field_of_view= self.field_of_view #rad
		self.data_SLIT.min_range=self.sensor_min_range
		self.data_SLIT.max_range=self.sensor_max_range
		self.data_SLIT.range=3.0
		self.data_SLIT.header.stamp = rospy.Time.now()
		self.data_SLIT.header.frame_id = "SLIT_frame"


		#SLDD
		self.data_SLDD=Range()
		self.data_SLDD.radiation_type=self.data_SLDD.ULTRASOUND #Ultrasound
		self.data_SLDD.field_of_view= self.field_of_view #rad
		self.data_SLDD.min_range=self.sensor_min_range
		self.data_SLDD.max_range=self.sensor_max_range
		self.data_SLDD.range=3.0
		self.data_SLDD.header.stamp = rospy.Time.now()
		self.data_SLDD.header.frame_id = "SLDD_frame"

		#Segundo mensaje
		#SLDT
		self.data_SLDT=Range()
		self.data_SLDT.radiation_type=self.data_SLDT.ULTRASOUND #Ultrasound
		self.data_SLDT.field_of_view= self.field_of_view #rad
		self.data_SLDT.min_range=self.sensor_min_range
		self.data_SLDT.max_range=self.sensor_max_range
		self.data_SLDT.range=3.0

		self.data_SLDT.header.stamp = rospy.Time.now()
		self.data_SLDT.header.frame_id = "SLDT_frame"


		#STI
		self.data_STI=Range()
		self.data_STI.radiation_type=self.data_STI.ULTRASOUND #Ultrasound
		self.data_STI.field_of_view= self.field_of_view #rad
		self.data_STI.min_range=self.sensor_min_range
		self.data_STI.max_range=self.sensor_max_range
		self.data_STI.range=3.0

		self.data_STI.header.stamp = rospy.Time.now()
		self.data_STI.header.frame_id = "STI_frame"


		#SLID
		self.data_SLID=Range()
		self.data_SLID.radiation_type=self.data_SLID.ULTRASOUND #Ultrasound
		self.data_SLID.field_of_view= self.field_of_view #rad
		self.data_SLID.min_range=self.sensor_min_range
		self.data_SLID.max_range=self.sensor_max_range
		self.data_SLID.range=3.0

		self.data_SLID.header.stamp = rospy.Time.now()
		self.data_SLID.header.frame_id = "SLID_frame"


		#SDD
		self.data_SDD=Range()
		self.data_SDD.radiation_type=self.data_SDD.ULTRASOUND #Ultrasound
		self.data_SDD.field_of_view= self.field_of_view #rad
		self.data_SDD.min_range=self.sensor_min_range
		self.data_SDD.max_range=self.sensor_max_range
		self.data_SDD.range=2.2

		self.data_SDD.header.stamp = rospy.Time.now()
		self.data_SDD.header.frame_id = "SDD_frame"

		self.max_array_ultrasonic=3

		self.STD_array=[0]*self.max_array_ultrasonic  #Max 15 elements
		self.SDI_array=[0]*self.max_array_ultrasonic
		self.SLIT_array=[0]*self.max_array_ultrasonic
		self.SLDD_array=[0]*self.max_array_ultrasonic
		self.i=0

		self.SLDT_array=[0]*self.max_array_ultrasonic
		self.STI_array=[0]*self.max_array_ultrasonic
		self.SLID_array=[0]*self.max_array_ultrasonic
		self.SDD_array=[0]*self.max_array_ultrasonic
		self.j=0


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
			var_enc.data = (encA)
			var_enc.encID=0

			self.enc_pub.publish(var_enc)

		elif msg.stdId == 258:
			#Encoder B Left
			(encB,) = struct.unpack('I',msg.data[0:4])
			(tencB,) = struct.unpack('I',msg.data[4:8])

			var_enc = enc_msg()
			var_enc.time = int(tencB)
			var_enc.data = (encB)
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
			#imu
			(ejez,) = struct.unpack('H', msg.data[6:8])
			(SJO,) = struct.unpack('H', msg.data[4:6])
			(ejex,) = struct.unpack('H', msg.data[2:4])
			(ejey,) = struct.unpack('H', msg.data[:2])


			data_imu=Imu()
			data_imu.header.stamp = rospy.Time.now()

			data_imu.linear_acceleration.x=ejex / self.kimu #[m/s^2]
			data_imu.linear_acceleration.y=ejey / self.kimu #[m/s^2]
			data_imu.linear_acceleration.y=ejez / self.kimu #[m/s^2]
			#If you have no estimate for one of the data elements  please set element 0 of the associated covariance matrix to -1
			data_imu.linear_acceleration_covariance[0] = -1

			self.pubSJO.publish(int(SJO))
			self.pubImu.publish(data_imu)

			#self.pubejex.publish(int(ejex))
			#self.pubejey.publish(int(ejey))
			#self.pubejez.publish(int(ejez))


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

			self.STD_pub.publish(self.data_STD)
			self.SDI_pub.publish(self.data_SDI)
			self.SLIT_pub.publish(self.data_SLIT)
			self.SLDD_pub.publish(self.data_SLDD)

			self.SLDT_pub.publish(self.data_SLDT)
			self.STI_pub.publish(self.data_STI)
			self.SLID_pub.publish(self.data_SLID)
			self.SDD_pub.publish(self.data_SDD)

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
