#!/usr/bin/env python


import math
from math import sin, cos, pi

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom.msg import enc_msg




class odom_class:
	def __init__(self):
		self.time_encA=0
		self.steps_encA=0
		self.time_encB=0
		self.steps_encB=0

		self.pasos_vuelta=64000.0
		self.radio=0.155
		self.D=0.525

	def main(self):

		rospy.init_node('odometry_publisher')
		rospy.loginfo("Nodo iniciado")

		#Publications
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
		self.odom_broadcaster = tf.TransformBroadcaster()
		rospy.loginfo('Publicaciones iniciadas')

		#Subscriber
		rospy.Subscriber("enc", enc_msg, self.callback) #One topic for both encoders

		self.send_odom()


	def send_odom(self):

		time_encA_last=self.time_encA
		time_encB_last=self.time_encB

		steps_encA_last=self.steps_encA
		steps_encB_last=self.steps_encB

		wd=0.0
		wi=0.0

		v=0.0
		omega=0.0

		vx=0.0
		vy=0.0
		vth=0.0

		x=0.0
		y=0.0
		th=0.0


		r = rospy.Rate(40.0)
		while not rospy.is_shutdown():


			current_time=rospy.Time.now()

			dtA=(self.time_encA-time_encA_last)*(10**-4) #100us
			dtB=(self.time_encB-time_encB_last)*(10**-4) #100us




			if((dtA!=0 and dtB!=0)):

				dtA=(self.time_encA-time_encA_last)*(10**-4) #100us
				dtB=(self.time_encB-time_encB_last)*(10**-4) #100us

				dstepsA=self.steps_encA-steps_encA_last
				dstepsB=self.steps_encB-steps_encB_last

				#Guardiar variables actuales
				time_encA_last=self.time_encA
				time_encB_last=self.time_encB

				steps_encA_last=self.steps_encA
				steps_encB_last=self.steps_encB


				#Velocidad angular

				wd=(dstepsA/self.pasos_vuelta)*2*pi/dtA
				wi=(dstepsB/self.pasos_vuelta)*2*pi/dtB



				#Cinematica inversa

				v=(wd+wi)*self.radio/2
				omega=(wd-wi)*self.radio/self.D



				#Dead reconing ---->Mirar si ambos tiempos son los mismos?
				vx =v*cos(th)
				vy =v*sin(th)
				vth =omega

				x +=vx*dtA
				y +=vy*dtA
				th +=vth*dtA



			# since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

			# first, we'll publish the transform over tf
			self.odom_broadcaster.sendTransform(
				(x, y, 0.),
				odom_quat,
				current_time,
				"base_link",
				"odom"
			)


			# next, we'll publish the odometry message over ROS
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"

			# set the position
			odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

			# set the velocity
			odom.child_frame_id = "base_link"
			odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

			# publish the message
			self.odom_pub.publish(odom)


			#Log datos

			#rospy.loginfo("tA " + str(time_encA_last) + " tB " + str(time_encB_last)
			#	+ " stepsA " + str(steps_encA_last) + " stepsB " + str(steps_encB_last)
			#	+ " dt " +str(dtA) + " dstep " +str(dstepsA)
			#	+ " wi "+ str(wi) + " wd " + str(wd)
			#	+ " v " +str(v) + " omega " + str(omega)
			#	+ " x " + str(x) + " y " + str(y) + " th " + str(th))






			r.sleep()



	#Read the encoder data
	def callback(self,msg):
		if(msg.encID==0): #EncoderA

			self.time_encA=msg.time
			self.steps_encA=msg.data

		if(msg.encID==1): #EncoderB

			self.time_encB=msg.time
			self.steps_encB=msg.data




if __name__ == '__main__':

	odom=odom_class()
	odom.main()
