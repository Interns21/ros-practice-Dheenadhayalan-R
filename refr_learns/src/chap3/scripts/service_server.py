#!/usr/bin/env python

import rospy
from chap3.srv import WordCount, WordCountResponse

def count_words(request):
	return WordCountResponse(len(request.word))

rospy.init_node('service_server')
service = rospy.Service('word_count', WordCount, count_words)
rospy.spin()
