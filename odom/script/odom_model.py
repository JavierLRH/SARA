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




#Read the encoder data
def callback(msg):
	if(msg.encID==0):
		#rospy.loginfo("lecturaA")
		global time_encA
		global steps_encA

		time_encA=msg.time
		steps_encA=msg.data

	if(msg.encID==1):
		#rospy.loginfo("lecturaB")
		global time_encB
		global steps_encB
		time_encB=msg.time
		steps_encB=msg.data






def send():

	pasos_vuelta=64000.0


	time_encA_last=time_encA
	time_encB_last=time_encB

	steps_encA_last=steps_encA
	steps_encB_last=steps_encB


	wd=0.0
	wi=0.0

	datoD=0
	datoI=0

	i=0

	rospy.Subscriber("enca", enc_msg, callback)
	rospy.Subscriber("encb", enc_msg, callback)

	r = rospy.Rate(5.0)
	while not rospy.is_shutdown():

		#Leer veocidades

		current_time=rospy.Time.now()

		dtA=(time_encA-time_encA_last)*(10**-4) #100us
		dtB=(time_encB-time_encB_last)*(10**-4) #100us
		dstepsA=steps_encA-steps_encA_last
		dstepsB=steps_encB-steps_encB_last

		#Guardiar variables actuales
		time_encA_last=time_encA
		time_encB_last=time_encB

		steps_encA_last=steps_encA
		steps_encB_last=steps_encB



		if(dtA!=0 and dtB!=0):
			#Velocidad angular

			wd=(dstepsA/pasos_vuelta)*2*pi/dtA
			wi=(dstepsB/pasos_vuelta)*2*pi/dtB


		#Enviar comandos
		datoD=int(data[i])
		datoI=int(data[i])
		i+=1


		msg = CAN()
		msg.stdId = 288
		msg.extId = -1
		msg.data = struct.pack('B', datoI) + struct.pack('B', datoD) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

		pub.publish( msg )




		#Log datos

		# rospy.loginfo("tA " + str(time_encA_last) + " tB " + str(time_encB_last)
		# 			  + " stepsA " + str(steps_encA_last) + " stepsB " + str(steps_encB_last)
		# 			  + " dt " +str(dtA) + " dstep " +str(dstepsA)
		# 			  + " wi "+ str(wi) + " wd " + str(wd)
		# 			  + " datoI " +str(datoI) + " datoD " + str(datoD))




		r.sleep()






if __name__ == '__main__':

	time_encA=0
	steps_encA=0
	time_encB=0
	steps_encB=0

	rospy.init_node('odometry_model')
	rospy.loginfo("Nodo iniciado")

	#Publications
	pub = rospy.Publisher('cantx', CAN, queue_size=100)


	F = open('consignas.txt','r') #carpeta personal
	line=F.readline()
	data=line.split(',')


	rospy.loginfo('subscripciones iniciadas')
	send()
