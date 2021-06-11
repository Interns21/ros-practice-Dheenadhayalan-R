#!/usr/bin/env python

import rospy
from chap3.msg import Complex
from random import random

rospy.init_node('message_publisher')
pub = rospy.Publisher('complex', Complex, queue_size=10)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
	msg = Complex()
	msg.real = random()
	msg.imaginary = random()
	rospy.loginfo(msg)
	
	pub.publish(msg)
	rate.sleep()
