#!/usr/bin/env python


import math
import struct
from math import sin, cos, pi

import rospy

import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom.msg import enc_msg
from odom.msg import CAN
from std_msgs.msg import Header




class odom_class:
	def __init__(self):

		self.pasos_vuelta=64000.0


		#Encoders
		self.time_encA=0
		self.steps_encA=0
		self.time_encB=0
		self.steps_encB=0

		self.time_encA_last=0
		self.time_encB_last=0

		self.steps_encA_last=0
		self.steps_encB_last=0


		self.posI=0.0
		self.posD=0.0

		self.wi=0.0
		self.wd=0.0





		#command
		self.comandD=0.0
		self.comandI=0.0

		self.datod=0
		self.datoi=0

		self.kdato=9.5

		self.twistTimeout = 2

		self.modoPC=0

	def main(self):

		rospy.init_node('hwinterface')
		rospy.loginfo("Nodo iniciado")

		#Publications
		self.data_pub = rospy.Publisher("odom_joint_state", JointState, queue_size=50)
		self.canpub = rospy.Publisher('cantx', CAN, queue_size=100)

		rospy.loginfo('Publicaciones iniciadas')

		#Subscriber
		rospy.Subscriber("enc", enc_msg, self.callback_odom) #One topic for both encoders
		rospy.Subscriber("joint_command", JointState, self.callback_vel)#Cambiar al mensaje recibido por el control
		rospy.Subscriber("canrx", CAN, self.callback_CAN)


		r = rospy.Rate(40.0)
		self.lastTwistTime = rospy.get_time()

		rospy.loginfo("Esperando sincronizacion")
		while not rospy.is_shutdown():
			self.check(self.modoPC)

			r.sleep()

	def check(self,modo):

			if(modo==3): #Sincronizado con la silla

				if (rospy.get_time() - self.lastTwistTime) > self.twistTimeout: #No he recbido datos en un tiempo, para la silla

					dato = CAN()
					dato.stdId = 288
					dato.extId = -1
					dato.data = struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

					self.canpub.publish(dato)

					rospy.loginfo("Parada por no recibir datos")

		#else:#No SINCRONIZADO
			#rospy.loginfo("La silla no esta sincronizada")


	def callback_vel(self,msg): #Recepcion de velocidades

		#Guardar variables
		self.comandD=msg.velocity[0] #rad/s
		self.comandI=msg.velocity[1] #rad/s

		#Ganancia

		self.datod=int(self.comandD*self.kdato)
		self.datoi=int(self.comandI*self.kdato)

		#Saturacion para evitar errores
		if self.datod>127:
			self.datod=127
		if self.datod<-127:
			self.datod=-127

		if self.datoi>127:
			self.datoi=127
		if self.datoi<-127:
			self.datoi=-127

		#Ajuste de dato negativo
		if  self.datod<0:
			self.datod=256+self.datod #Maxima velocidad 129, 128 parado

		if  self.datoi<0:
			self.datoi=256+self.datoi

		if(self.modoPC==3): #Sincronizado con la silla,permito enviar datos
			dato = CAN()
			dato.stdId = 288
			dato.extId = -1
			dato.data = struct.pack('B', self.datod) + struct.pack('B', self.datoi) + struct.pack('B', 0) + struct.pack('B', 0) + 'C' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

			self.canpub.publish(dato)


		#Log datos

		#rospy.loginfo("vx " + str(self.vx) + " vth " + str(self.vth)
				#	  + " wd " + str(self.wd) + " wi " + str(self.wi)
				#	  + " datod " +str(self.datod) + " datoi " +str(self.datoi))

		self.lastTwistTime = rospy.get_time() #Actualizar tiempo ultima recepcion de datos

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



	#Read the encoder data
	def callback_odom(self,msg):
		if(msg.encID==0): #EncoderA

			self.time_encA=msg.time
			self.steps_encA=msg.data

			#Calcular incrementos
			dtA=(self.time_encA-self.time_encA_last)*(10**-4) #100us
			dstepsA=self.steps_encA-self.steps_encA_last

			#Guardiar variables actuales
			self.time_encA_last=self.time_encA
			self.steps_encA_last=self.steps_encA

			#Posicion
			self.posD=(dstepsA/self.pasos_vuelta)*2*pi
			#Velocidad angular
			self.wd=self.posD/dtA




		if(msg.encID==1): #EncoderB

			self.time_encB=msg.time
			self.steps_encB=msg.data

			#Calcular incrementos
			dtB=(self.time_encB-self.time_encB_last)*(10**-4) #100us
			dstepsB=self.steps_encB-self.steps_encB_last

			#Guardiar variables actuales
			self.time_encB_last=self.time_encB
			self.steps_encB_last=self.steps_encB

			#Distancia
			self.posI=(dstepsB/self.pasos_vuelta)*2*pi
			#Velocidad angular
			self.wi=self.posI/dtB



		##Publications
		data=JointState()
		data.name=("D","I")
		data.position=(self.posD, self.posI)
		data.velocity=(self.wd, self.wi)
		data.effort=(0.0,0.0)

		data.header.stamp = rospy.Time.now()
		data.header.frame_id = "base_link"





		self.data_pub.publish(data)



if __name__ == '__main__':

	odom=odom_class()
	odom.main()
