#!/usr/bin/env python

#####Creador Francisco Javier La Roda######
#####Funcion: Interface entre el controlador hw_interface y el bus can
#####Recibe: Controlador [Joint_State (commandD, commandI)],Can [Datos Encoder]
####Envia: Controlador[PosD, PosI, Wi, Wd], Can [DatoI, DatoD]
import math
import struct
from math import sin, cos, pi

import rospy

import tf

##Mensajes
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sara_control.msg import enc_msg
from sara_control.msg import CAN
from std_msgs.msg import Header




class odom_class:
	def __init__(self):

		#parametros constantes
		self.pasos_vuelta=64000.0
		self.left= 0
		self.right= 1

		self.kdato=9.5
		self.twistTimeout = 2


		#Encoders
		self.time_enc_left=0
		self.time_enc_right=0

		self.steps_enc_left=0
		self.steps_enc_right=0

		self.time_enc_left_last=0
		self.time_enc_right_last=0

		self.steps_enc_left_last=0
		self.steps_enc_right_last=0


		self.posI=0.0
		self.posD=0.0

		self.wi=0.0
		self.wd=0.0





		#command
		self.comandD=0.0
		self.comandI=0.0

		self.datod=0
		self.datoi=0



		self.modoPC=0

	def main(self):

		rospy.init_node('hwinterface')
		rospy.loginfo("Iniciado nodo SARA_interface")

		#Publications
		self.data_pub = rospy.Publisher("odom_joint_state", JointState, queue_size=50)
		self.canpub = rospy.Publisher('cantx', CAN, queue_size=100)

		#rospy.loginfo('Publicaciones iniciadas')

		#Subscriber
		rospy.Subscriber("enc", enc_msg, self.callback_odom) #One topic for both encoders
		rospy.Subscriber("cmd_pub", JointState, self.callback_vel)#Cambiar al mensaje recibido por el control
		rospy.Subscriber("canrx", CAN, self.callback_CAN)

		rospy.loginfo("Esperando sincronizacion")

		#Bucle que para la silla si no se reciben velocidades en un tiempo
		r = rospy.Rate(5.0)
		self.lastTwistTime = rospy.get_time()
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

					#rospy.loginfo_throttle(5, "Parada por no recibir datos")
					rospy.loginfo_once("Parada por no recibir datos")


		#else:#No SINCRONIZADO
			#rospy.loginfo("La silla no esta sincronizada")


	def callback_vel(self,msg): #Recepcion de velocidades

		#Guardar variables
		self.comandD=msg.velocity[self.right] #rad/s
		self.comandI=msg.velocity[self.left] #rad/s

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
		if(msg.encID==0): #EncoderA (RIGHT)

			self.time_enc_right=msg.time
			self.steps_enc_right=msg.data

			#Calcular incrementos
			dt_right=(self.time_enc_right-self.time_enc_right_last)*(10**-4) #100us
			dsteps_right=self.steps_enc_right-self.steps_enc_right_last

			#Guardiar variables actuales
			self.time_enc_right_last=self.time_enc_right
			self.steps_enc_right_last=self.steps_enc_right

			#Posicion
			self.posD=(self.steps_enc_right/self.pasos_vuelta)*2*pi
			#Velocidad angular
			self.wd=((dsteps_right/self.pasos_vuelta)*2*pi)/dt_right

			#Save data to publish
			data=JointState()
			data.name= ["RIGHT"]
			data.position=[self.posD]
			data.velocity=[self.wd]
			data.effort=[0]
			data.header.stamp = rospy.Time.from_sec(self.time_enc_right_last*(10**-4)) #rospy.Time.now()
			data.header.frame_id = "base_link"




		elif(msg.encID==1): #EncoderB (LEFT)

			self.time_enc_left=msg.time
			self.steps_enc_left=msg.data

			#Calcular incrementos
			dt_left=(self.time_enc_left-self.time_enc_left_last)*(10**-4) #100us
			dsteps_left=self.steps_enc_left-self.steps_enc_left_last

			#Guardiar variables actuales
			self.time_enc_left_last=self.time_enc_left
			self.steps_enc_left_last=self.steps_enc_left

			#Distancia
			self.posI=(self.steps_enc_left/self.pasos_vuelta)*2*pi
			#Velocidad angular
			self.wi=((dsteps_left/self.pasos_vuelta)*2*pi)/dt_left

			#Save data to publish
			data=JointState()
			data.name= ["LEFT"]
			data.position=[self.posI]
			data.velocity=[self.wi]
			data.effort=[0]
			data.header.stamp =rospy.Time.from_sec(self.time_enc_left_last*(10**-4)) #rospy.Time.now()
			data.header.frame_id = "base_link"



		#Publish topic
		self.data_pub.publish(data)



if __name__ == '__main__':

	odom=odom_class()
	odom.main()
