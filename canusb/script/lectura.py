#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import yaml
import roslib
roslib.load_manifest("canusb");
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Int8

from odom.msg import enc_msg

import rospy
from _CAN import CAN

import serial
import struct
import time
import threading


def callback(msg):

	if msg.stdId == 272:
		(modo,) = struct.unpack('H',msg.data[4:6])
		(joyx,) = struct.unpack('H',msg.data[2:4])
		(joyy,) = struct.unpack('H',msg.data[6:8])
		(velD,) = struct.unpack('B',msg.data[0:1])
		(velI,) = struct.unpack('B',msg.data[1:2])

		pubmodo.publish(int(modo))
		pubjoyx.publish(int(joyx))
		pubjoyy.publish(int(joyy))
		pubvelD.publish(int(velD))
		pubvelI.publish(int(velI))

	elif msg.stdId == 273:
		(modoPC,) = struct.unpack('B',msg.data[0:1])

		pubmodoPC.publish(int(modoPC))

	elif msg.stdId == 257:
		#Encoder A Derecha
		(encA,) = struct.unpack('I',msg.data[0:4])
		(tencA,) = struct.unpack('I',msg.data[4:8])

		var_enc = enc_msg()
		var_enc.time = int(tencA)
		var_enc.data = int(encA)
		var_enc.encID=0

		enc_pub.publish(var_enc)

	elif msg.stdId == 258:
		#Encoder B Izquierda
		(encB,) = struct.unpack('I',msg.data[0:4])
		(tencB,) = struct.unpack('I',msg.data[4:8])

		var_enc = enc_msg()
		var_enc.time = int(tencB)
		var_enc.data = int(encB)
		var_enc.encID=1

		enc_pub.publish(var_enc)

	elif msg.stdId == 528:
		(bat,) = struct.unpack('B',msg.data[0:1])

		pubbat.publish(int(bat))

	elif msg.stdId == 513:
		(STD,) = struct.unpack('H', msg.data[:2])
		(SDI,) = struct.unpack('H', msg.data[2:4])
		(SLIT,) = struct.unpack('H', msg.data[4:6])
		(SLDD,) = struct.unpack('H', msg.data[6:8])

		pubSLIT.publish(int(SLIT))
		pubSLDD.publish(int(SLDD))
		pubSTD.publish(int(STD))
		pubSDI.publish(int(SDI))


	elif msg.stdId == 514:

		(SLDT,) = struct.unpack('H', msg.data[6:8])
		(STI,) = struct.unpack('H', msg.data[4:6])
		(SLID,) = struct.unpack('H', msg.data[2:4])
		(SDD,) = struct.unpack('H', msg.data[:2])

		pubSLID.publish(int(SLID))
		pubSDD.publish(int(SDD))
		pubSLDT.publish(int(SLDT))
		pubSTI.publish(int(STI))

	elif msg.stdId == 515:
		(ejez,) = struct.unpack('H', msg.data[6:8])
		(SJO,) = struct.unpack('H', msg.data[4:6])
		(ejex,) = struct.unpack('H', msg.data[2:4])
		(ejey,) = struct.unpack('H', msg.data[:2])

		pubSJO.publish(int(SJO))
		pubejex.publish(int(ejex))
		pubejey.publish(int(ejey))
		pubejez.publish(int(ejez))

def listener():
    rospy.Subscriber("canrx", CAN, callback)
    rospy.spin()

if __name__ == '__main__':
	rospy.init_node('lectura', anonymous=True)

	rospy.loginfo("Crea publisher")

	pubmodo = rospy.Publisher('/modo', Int16, queue_size=100)
	pubjoyx = rospy.Publisher('/joyx', Int16, queue_size=100)
	pubjoyy = rospy.Publisher('/joyy', Int16, queue_size=100)
	pubvelD = rospy.Publisher('/velD', Int16, queue_size=100)
	pubvelI = rospy.Publisher('/velI', Int16, queue_size=100)
	pubmodoPC = rospy.Publisher('/modoPC', Int16, queue_size=100)
	enc_pub = rospy.Publisher("enc", enc_msg, queue_size=50) #1 topic for both encoders
	pubbat = rospy.Publisher('/bat', Int16, queue_size=100)
	pubSLIT = rospy.Publisher('/SLIT', Int16, queue_size=100)
	pubSLDD = rospy.Publisher('/SLDD', Int16, queue_size=100)
	pubSTD = rospy.Publisher('/STD', Int16, queue_size=100)
	pubSDI = rospy.Publisher('/SDI', Int16, queue_size=100)
	pubSLID = rospy.Publisher('/SLID', Int16, queue_size=100)
	pubSDD = rospy.Publisher('/SDD', Int16, queue_size=100)
	pubSLDT = rospy.Publisher('/SLDT', Int16, queue_size=100)
	pubSTI = rospy.Publisher('/STI', Int16, queue_size=100)
	pubSJO = rospy.Publisher('/SJO', Int16, queue_size=100)
	pubejex = rospy.Publisher('/ejex', Int16, queue_size=100)
	pubejey = rospy.Publisher('/ejey', Int16, queue_size=100)
	pubejez = rospy.Publisher('/ejez', Int16, queue_size=100)

	try:
		listener()
	except rospy.ROSInterruptException: pass
