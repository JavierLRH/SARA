#!/usr/bin/env python


import math
import struct
from math import sin, cos, pi

import rospy

import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom.msg import CAN


class BaseController_Class:
	def __init__(self):
		self.pasos_vuelta=64000.0
		self.radio=0.155
		self.D=0.525

		self.wd=0.0
		self.wi=0.0


		self.vx=0.0
		self.vth=0.0

		self.datod=0
		self.datoi=0

		self.kdato=9.5

		self.twistTimeout = 2

		self.modoPC=0

	def main(self):
		rospy.init_node('cinematic')
		rospy.loginfo("Nodo iniciado")

		#Publications
		self.canpub = rospy.Publisher('cantx', CAN, queue_size=100)

		#Subscription
		rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
		rospy.Subscriber("canrx", CAN, self.callback_CAN)


		r = rospy.Rate(20.0)
		self.lastTwistTime = rospy.get_time()

		rospy.loginfo("Esperando sincronizacion")
		while not rospy.is_shutdown():
			self.publish(self.modoPC)

			r.sleep()



	def publish(self,modo):
		#Publicar datos

		if(modo==3): #Sincronizado con la silla, envio datos
			if (rospy.get_time() - self.lastTwistTime) < self.twistTimeout:
				dato = CAN()
				dato.stdId = 288
				dato.extId = -1
				dato.data = struct.pack('B', self.datod) + struct.pack('B', self.datoi) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

				self.canpub.publish(dato)
			else: #Si no recibo datos de velocidades durante un tiempo, por seguridad para la silla.
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
		self.vx=msg.linear.x
		self.vth=msg.angular.z


		#Cinematica
		self.wd=(self.vx/self.radio)+((self.vth*self.D)/(2*self.radio))
		self.wi=(self.vx/self.radio)-((self.vth*self.D)/(2*self.radio))

		#Saturacion
		if self.wd>10:
			self.wd=10
		if self.wd<-10:
			self.wd=-10

		if self.wi>10:
			self.wi=10
		if self.wi<-10:
			self.wi=-10
		#Ganancia

		self.datod=int(self.wd*self.kdato)
		self.datoi=int(self.wi*self.kdato)

		#Ajuste de dato negativo
		if  self.datod<0:
			self.datod=255+self.datod

		if  self.datoi<0:
			self.datoi=255+self.datoi


		#Log datos

		#rospy.loginfo("vx " + str(self.vx) + " vth " + str(self.vth)
					#  + " wd " + str(self.wd) + " wi " + str(self.wi)
					#  + " datod " +str(self.datod) + " datoi " +str(self.datoi))

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


if __name__ == '__main__':

	BaseController=BaseController_Class()
	BaseController.main()
