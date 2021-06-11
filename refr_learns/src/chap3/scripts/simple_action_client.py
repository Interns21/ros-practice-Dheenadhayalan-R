#!/usr/bin/env python

import rospy
import actionlib
from chap3.msg import timerAction, timerGoal, timerResult

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', timerAction)
client.wait_for_server()
goal = timerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)
client.send_goal(goal)
client.wait_for_result()
print('Time alpsed: %f'%(client.get_result().time_elapsed.to_sec()))
