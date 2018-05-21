#!/usr/bin/env python


import math
import struct
from math import sin, cos, pi

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom.msg import enc_msg
from odom.msg import CAN




class vel_test_class:
	def __init__(self):

		self.i=0

		self.wd=0.0
		self.wi=0.0

		self.pasos_vuelta=64000.0


		self.datoD=0
		self.datoI=0

		self.steps_encA_last=0
		self.steps_encB_last=0

		self.time_encA_last=0
		self.time_encB_last=0



		self.modoPC=0

	def main(self):
		rospy.init_node('vel_test')


		#Publications
		self.canpub = rospy.Publisher('cantx', CAN, queue_size=100)

		#Subscription
		rospy.Subscriber("canrx", CAN, self.callback_CAN)
		rospy.Subscriber("enc",enc_msg, self.callback)

		rospy.loginfo('subscripciones iniciadas')


		#Leer ficheros
		F = open('consignas.txt','r') #carpeta personal
		line=F.readline()
		self.data=line.split(',')

		F = open('consignas2.txt','r') #carpeta personal
		line=F.readline()
		self.data2=line.split(',')




		r = rospy.Rate(5.0)

		rospy.loginfo("Esperando sincronizacion")
		while not rospy.is_shutdown():
			self.check(self.modoPC)

			r.sleep()



	def check(self,modo):

			if(modo==3): #Sincronizado con la silla, enviar datos de prueba
				#Leer veocidades

				current_time=rospy.Time.now()


				#Enviar comandos
				self.datoD=int(self.data[self.i])
				self.datoI=int(self.data2[self.i])
				self.i+=1

				#rospy.loginfo("i= " + str(self.i))


				msg = CAN()
				msg.stdId = 288
				msg.extId = -1
				msg.data = struct.pack('B', self.datoI) + struct.pack('B', self.datoD) + struct.pack('B', 0) + struct.pack('B', 0) + 'C' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

				self.canpub.publish( msg )


				if(self.i>=39): #Parada
					modo==0
					msg = CAN()
					msg.stdId = 288
					msg.extId = -1
					msg.data = struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + 'C' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

					self.canpub.publish( msg )




	#Read the encoder data
	def callback(self,msg):
		if(msg.encID==0): #EncoderA

			self.time_encA=msg.time
			self.steps_encA=msg.data

			dtA=(self.time_encA-self.time_encA_last)*(10**-4) #100us
			dstepsA=self.steps_encA-self.steps_encA_last

			self.time_encA_last=self.time_encA
			self.steps_encA_last=self.steps_encA

			self.wd=(dstepsA/self.pasos_vuelta)*2*pi/dtA

		if(msg.encID==1): #EncoderB

			self.time_encB=msg.time
			self.steps_encB=msg.data

			dtB=(self.time_encB-self.time_encB_last)*(10**-4) #100us
			dstepsB=self.steps_encB-self.steps_encB_last

			self.time_encB_last=self.time_encB
			self.steps_encB_last=self.steps_encB

			self.wi=(dstepsB/self.pasos_vuelta)*2*pi/dtB

		#Log de los datos
		rospy.loginfo("wd " + str(self.wd) + " wi " + str(self.wi)
				+ " datoD " + str(self.datoD) + " datoI " + str(self.datoI))



	def callback_CAN(self,msg): #Recepcion de sincronizacion del bus can
		if msg.stdId == 273:
			(self.modoPC,)= struct.unpack('B', msg.data[:1])
			self.sincronizar(self.modoPC)

	def sincronizar(self, sinc):
		if sinc == 3:
			msg = CAN()
			msg.stdId = 288
			msg.extId = -1
			msg.data = struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)
			rospy.loginfo("Sincronizado")


if __name__ == '__main__':

	vel_test=vel_test_class()
	vel_test.main()
