#!/usr/bin/env python

import rospy
import time
import actionlib
from chap3.msg import timerAction, timerGoal, timerResult, timerFeedback

class FancyServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('timer', timerAction, self.execute, False)
		self.server.start()

	def execute(self, request):
                start_time = time.time()
                update_count = 0
                if request.time_to_wait.to_sec() > 60.0:
                        result = timerResult()
                        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                        result.updates_sent = update_count
                        self.server.set_aborted(result, 'Timer aborted due to too-long wait')
                        return
                while (time.time() - start_time) < request.time_to_wait.to_sec():
                        if self.server.is_preempt_requested():
                                result = timerResult()
                                result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                                result.updates_sent = update_count
                                self.server.set_preempted(result, 'Timer preempted')
                                return
                        feedback = timerFeedback()
                        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                        feedback.time_remaining = request.time_to_wait - feedback.time_elapsed
                        self.server.publish_feedback(feedback)
                        update_count += 1

                        time.sleep(0.1)

                result = timerResult()
                result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
                result.updates_sent = update_count
                self.server.set_succeeded(result, 'Timer completed successfully')


if __name__ == '__main__':
        rospy.init_node('timer_action_server')
        server = FancyServer()
        rospy.spin()
