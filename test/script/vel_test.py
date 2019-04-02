#!/usr/bin/env python

import rospy
import struct
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Header




class vel_test_class:
	def __init__(self):
		self.i=0


	def main(self):
		rospy.init_node('w_test')

		rospy.loginfo("Nodo iniciado")


		#Publications
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)


		#Leer ficheros
		file_name = rospy.get_param("~fileV", None)

		F = open(file_name,'r') #carpeta personal
		line=F.readline()
		self.dataV=line.split(',')
		F.close()

		file_name = rospy.get_param("~filew", None)

		F = open(file_name,'r') #carpeta personal
		line=F.readline()
		self.dataw=line.split(',')
		F.close()


		r = rospy.Rate(10.0) #100ms

		while not rospy.is_shutdown():
			#Enviar comandos

			V=float(self.dataV[self.i])
			w=float(self.dataw[self.i])
			self.i+=1


			data=Twist()
			data.linear.x=V
			data.angular.z=w
			#data.header.stamp = rospy.Time.now()
			#data.header.frame_id = "base_link"

			self.cmd_pub.publish(data)

			r.sleep()



if __name__ == '__main__':

	vel_test=vel_test_class()
	vel_test.main()
