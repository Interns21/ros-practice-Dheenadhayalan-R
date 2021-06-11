#!/usr/bin/env python3.9

import rospy
import time
import actionlib
import sys
from chap3.msg import timerAction, timerGoal

class FancyClient:
    def __init__(self, wait_time):
        self.client = actionlib.SimpleActionClient('timer', timerAction)
        self.client.wait_for_server()
        goal = timerGoal()
        goal.time_to_wait = rospy.Duration.from_sec(wait_time)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)

        self.client.wait_for_result()
        self.print_result(self.client.get_state(), self.client.get_goal_status_text(), self.client.get_result())

    def feedback_cb(self, feedback):
        print('Time elapsed:', feedback.time_elapsed.to_sec())
        print('Time remining:', feedback.time_remaining.to_sec())

    def print_result(self, state, status, result):
        print('State:',state)
        print('Status:',status)
        print('Time elapsed:', result.time_elapsed.to_sec())
        print('Update sent:', result.updates_sent)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage:')
        print('fancy_action_client.py [time_to_wait]')
        sys.exit(0)

    rospy.init_node('timer_action_client')
    client = FancyClient(int(sys.argv[1]))
