#!/usr/bin/env python


import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from odom.msg import enc_msg




#Read the encoder data
#Git
def callback(msg):
	if(msg.encID==0):
		#rospy.loginfo("lecturaA")

		time_encA=msg.time
		steps_encA=msg.data

		rospy.loginfo("time_encA " + str(time_encA) + " steps_encA " + str(steps_encA))



	if(msg.encID==1):
		#rospy.loginfo("lecturaB")

		time_encB=msg.time
		steps_encB=msg.data

		rospy.loginfo("time_encB " + str(time_encB) + " steps_encB " + str(steps_encB))








def listener():




	rospy.Subscriber("enca", enc_msg, callback)
	rospy.Subscriber("encb", enc_msg, callback)
	rospy.spin()








if __name__ == '__main__':

	rospy.init_node('odometry_log')
	rospy.loginfo("Nodo iniciado")



	listener()
