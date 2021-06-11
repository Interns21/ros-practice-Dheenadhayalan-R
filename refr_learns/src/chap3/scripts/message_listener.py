#!/usr/bin/env python3.9

import rospy
from chap3.msg import Complex

def callback(msg):
	print('Real part:', msg.real)
	print('Complex part:', msg.imaginary)

def listener():
	rospy.init_node('message_listener')
	rospy.Subscriber('complex', Complex, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
