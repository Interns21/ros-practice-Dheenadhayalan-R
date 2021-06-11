#!/usr/bin/env python

import rospy
import time
import actionlib
from chap3.msg import timerAction, timerGoal, timerResult

def do_timer(goal):
	start_time = time.time()
	time.sleep(goal.time_to_wait.to_sec())
	result = timerResult()
	result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
	result.updates_sent = 0
	server.set_succeeded(result)

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', timerAction, do_timer, False)
server.start()
rospy.spin()

