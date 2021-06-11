#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TwistConverter:
    def __init__(self):
        """
        Initialize publishers, subscribers, common variables and parameters
        publish Twist message of a key press in 'cmd_vel' topic
        """
        self.target_vel = self.current_vel = Twist()
        self.vel_scales = [0.1, 0.1]
        self.vel_accels = [1.0, 1.0]
        self.key_mapping = {'w': [1, 0], 'x': [-1, 0], 'a': [0, 1], 'd': [0, -1], 's': [0, 0]}  # linear and angular velocities' direction for respective key press
        self.vel_scales[0] = self.fetch_params('~linear_scale', self.vel_scales[0])  # obtaining scales from commandline or default value
        self.vel_scales[1] = self.fetch_params('~angular_scale', self.vel_scales[1])
        self.vel_accels[0] = self.fetch_params('~linear_accel', self.vel_accels[0])  # obtaining accleration from commandline or default value
        self.vel_accels[1] = self.fetch_params('~angular_accel', self.vel_accels[1])

        self.key_sub = rospy.Subscriber('keys', String, self.key_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.cmd_vel_pub.publish(self.current_vel)
        self.last_pub_time = rospy.Time.now()
        self.execute()

    def execute(self):
        """
        Execution of teleop
        """
        print('Publishing Twist messages press Ctrl-C to exit...')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.reach_vel(self.target_vel)
            rate.sleep()

    def fetch_params(self,name, default):
        """
        Returns parameters from command line if provided or returns default values
        """
        if rospy.has_param(name):  # return given parameter else default parameter
            return rospy.get_param(name)
        else:
            rospy.logwarn('%s scale not provided, using default value %.1f'%\
            (name, default))  # Warn if parameter not provided
            return default

    def key_callback(self, key):
        """
        Callback for key press messages and sets target Twist
        """
        if key.data[0] in self.key_mapping:  # if the pressed key is in w, a, s, d, x
            vels = self.key_mapping[key.data[0]]  # obtain respective velocity direction
            cmd = Twist()
            cmd.linear.x = vels[0] * self.vel_scales[0]  # Scale the velocities
            cmd.angular.z = vels[1] * self.vel_scales[1]
            self.target_vel = cmd  # set target Twist

    def reach_vel(self, target_vel):
        """
        Publishes the next Twist and get ready for next loop
        """
        current_time = rospy.Time.now()

        next_vel = Twist()

        next_vel.linear.x = self.next_vel(self.current_vel.linear.x,
        target_vel.linear.x,
        current_time,
        self.last_pub_time,
        self.vel_accels[0])  # set next linear velocity to publish

        next_vel.angular.z = self.next_vel(self.current_vel.angular.z,
        target_vel.angular.z,
        current_time,
        self.last_pub_time,
        self.vel_accels[1])  # set next angular velocity to publish

        self.cmd_vel_pub.publish(next_vel)  # publish next_vel and get ready for next loop
        self.last_pub_time = current_time
        self.current_vel = next_vel

    def next_vel(self, current_vel, target_vel, current_time, last_time, accel):
        """
        Checks if the target is reached and gives the next velocity accordingly
        considering current velocity, target velocity, acclerations and
        time elapsed
        """
        if current_vel == target_vel:  # if target velocity reached return same
            return target_vel
        delta_sign = 1.0 if target_vel > current_vel else -1.0  # check whether to acclerate or decelerate
        delta_vel = accel * (current_time - last_time).to_sec()  # obtain the magnitude of velocity change using acceleration and time passed
        vel_next = current_vel + delta_sign * delta_vel  # the next expected velocity

        if delta_sign * vel_next >= delta_sign * target_vel :  # check if reached target
            return target_vel
        else:
            return vel_next


if __name__ == '__main__':
    rospy.init_node('key_to_twist')
    key2Twist = TwistConverter()
