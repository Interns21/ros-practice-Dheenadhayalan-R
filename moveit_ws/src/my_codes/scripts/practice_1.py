#!/usr/bin/env python

import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

class Practice1:
    def __init__(self, argv):
        moveit_commander.roscpp_initialize(argv)
        self.robot = moveit_commander.RobotCommander()
        self.group_name = 'panda_arm'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.execute()

    def execute(self):
        orient = [Quaternion(*tf.transformations.quaternion_from_euler(pi, -pi/2, -pi/2)),
        Quaternion(*tf.transformations.quaternion_from_euler(pi, -pi/2, -pi/2))]

        pose = [Pose(Point(0.25, -0.35, 0.7), orient[0]),
        Pose(Point(0.25, 0.25, 0.7), orient[1])]

        while not rospy.is_shutdown():
            pose[0].position.y = 0.5 + random.uniform(-0.1, 0.1)
            pose[0].position.y = -0.5 + random.uniform(-0.1, 0.1)
            for side in [0,1]:
                self.move_group.set_pose_target(pose[side])
                self.move_group.go(wait=True)

        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('practice_code_1_wave')
    Practice1(sys.argv)
