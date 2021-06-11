#! /usr/bin/env python

import sys
import copy
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import moveit_commander
from moveit_commander.conversions import pose_to_list
from math import pi
import time


class MyMove:
    def __init__(self):
        print 'Moveit commander: Initialisation...'
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('my_move_group_interface', anonymous=True)
        print 'Moveit commander: Robot commander'
        self.robot = moveit_commander.RobotCommander()
        print 'Moveit commander: Planning scent interface'
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = 'panda_arm'
        print 'Movit commander: Move group commander'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        print 'Sleeping for 3 seconds'
        time.sleep(3)
        self.init_config_state()
        self.execute()

    def init_config_state(self):
        print 'Into init_config_state and obtain current config'
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        print 'Move group: Go'
        self.move_group.go(joint_goal, wait=True)

        print 'Move group: stop'
        self.move_group.stop()

    def execute(self):
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_to_goal(pose_goal)

    def loop_input(self):
        print 'Enter input'
        a = input()
        while a != 'q':
            coord = [x for x in a.split(',')]
            point = Point(*tuple(coord[:3]))
            orient = (pi*coord[3], pi*coord[4], pi*coord[5])
            orient = Quaternion(*quaternion_from_euler(*orient))
            pose = Pose(point, orient)
            self.move_to_goal(pose)
            a = input()

    def move_to_goal(self, goal):
        self.move_group.set_pose_target(goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


if __name__ == '__main__':
    MyMove()
