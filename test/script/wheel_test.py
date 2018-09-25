#!/usr/bin/env python

import rospy
import struct
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header




class vel_test_class:
	def __init__(self):
		self.i=0


	def main(self):
		rospy.init_node('w_test')

		rospy.loginfo("Nodo iniciado")


		#Publications
		self.cmd_pub = rospy.Publisher('/cmd_wheel', JointState, queue_size=100)


		#Leer ficheros
		file_name = rospy.get_param("~fileWL", None)

		F = open(file_name,'r') #carpeta personal
		line=F.readline()
		self.dataWL=line.split(',')
		F.close()

		file_name = rospy.get_param("~fileWR", None)

		F = open(file_name,'r') #carpeta personal
		line=F.readline()
		self.dataWR=line.split(',')
		F.close()


		r = rospy.Rate(1.0) #100ms

		while not rospy.is_shutdown():
			#Enviar comandos

			WR=int(self.dataWR[self.i])
			WL=int(self.dataWL[self.i])
			self.i+=1

			
			data=JointState()
			data.name= ["RIGHT", "LEFT"]
			data.position=[0, 0]
			data.velocity=[WR, WL]
			data.effort=[0, 0]
			data.header.stamp = rospy.Time.now()
			data.header.frame_id = "base_link"

			self.cmd_pub.publish(data)

			r.sleep()



if __name__ == '__main__':

	vel_test=vel_test_class()
	vel_test.main()
