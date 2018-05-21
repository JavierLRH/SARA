#!/usr/bin/env python


import rospy

import tf
from odom.msg import enc_msg




def send():


	steps_encA=0
	steps_encB=0
	time=0


	r = rospy.Rate(5.0) #Probando
	while not rospy.is_shutdown():
		#rospy.loginfo("Enviar parametros")

		time+=1
		steps_encA+=2
		steps_encB+=1





		var_encA = enc_msg()
		var_encA.time = time
		var_encA.data = steps_encA
		var_encA.encID=0

		# publish the message
		enc_pub.publish(var_encA)

		var_encB = enc_msg()
		var_encB.time = time
		var_encB.data = steps_encB
		var_encB.encID=1


		enc_pub.publish(var_encB)



		r.sleep()






if __name__ == '__main__':

	rospy.init_node('odometry_test')
	rospy.loginfo("Nodo iniciado")

	#Publications
	enc_pub = rospy.Publisher("enc", enc_msg, queue_size=50)





	send()
