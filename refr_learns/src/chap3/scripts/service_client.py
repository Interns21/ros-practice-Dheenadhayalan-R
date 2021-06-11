#!/usr/bin/env python3.9

import rospy
from chap3.srv import WordCount
import sys

rospy.init_node('service_client')
rospy.wait_for_service('word_count')

word_counter = rospy.ServiceProxy('word_count', WordCount)

word = sys.argv[1]

word_count = word_counter(word)

print('Count:',word_count.count)
