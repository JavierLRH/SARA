#!/usr/bin/env python

#####Creador Francisco Javier La Roda######
#####Funcion: Probar el funcionamiento de hw_interface
####Envia: Controlador[PosD, PosI, Wi, Wd]
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



		self.posI_last=0.0
		self.posD_last=0.0

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

		rospy.init_node('joint_state_pub_test')
		rospy.loginfo("Nodo iniciado")

		#Publications
		self.data_pub = rospy.Publisher("odom_joint_state", JointState, queue_size=50)
		self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

		rospy.loginfo('Publicaciones iniciadas')




		r = rospy.Rate(5.0)

		while not rospy.is_shutdown():
			self.send()

			r.sleep()

	def send(self):

		#Publico info de las ruedas

		self.posD=self.posD+0.1
		self.posI=self.posI+0.1

		self.wd=(self.posD-self.posD_last)/5
		self.wi=(self.posI-self.posI_last)/5

		##Publications
		data=JointState()
		data.name=("D","I")
		data.position=(self.posD, self.posI)
		data.velocity=(self.wd, self.wi)
		data.effort=(0.0,0.0)

		data.header.stamp = rospy.Time.now()
		data.header.frame_id = "base_link"


		self.data_pub.publish(data)

		self.posD_last=self.posD;
		self.posI_last=self.posI;

		#Publico velocidad deseada
		vel=Twist()
		vel.linear.x=4.0
		vel.angular.z=0.0
		self.cmd_pub.publish(vel)

if __name__ == '__main__':

	odom=odom_class()
	odom.main()
