#! /usr/bin/env python

import sys, rospy, tf, moveit_commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi


class ArmManip:
    def __init__(self):
        self.eul_to_quat = tf.transformations.quaternion_from_euler
        self.quat_to_eul = tf.transformations.euler_from_quaternion
        self.group_name = 'panda_arm'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def move_to(self,point=None,orient=None):
        goal_pose = self.move_group.get_current_pose().pose
        if point is not None:
            current_pose.position.x = point.x
            current_pose.position.y = point.y
            current_pose.position.z = point.z
        if orient is not None:
            current_pose.orientation.x = orient.x
            current_pose.orientation.y = orient.y
            current_pose.orientation.z = orient.z
            current_pose.orientation.w = orient.w

        self.move_group.set_pose_target(goal_pose)
        self.move_group.go(wait=True)
        move_group.stop()

    def man_control(self):
        control_keys = {'q': 1, 'w': -1,
        'e': 2, 'r': -2,
        't': 3, 'y': -3,
        'a': 4, 's': -4,
        'd': 5, 'f': -5,
        'g': 6, 'h': -6}
        key_sub = rospy.Subscriber('keys', String, self.man_control_callback, control_keys)
        print 'Start pressing keys in key publisher'
        rospy.spin()

    def man_control_callback(self, key, control_keys):
        position_step = 0.05
        orientation_step = pi/20
        if key.data[0] in control_keys:
            key_factor = control_keys[key.data[0]]
            if key_factor > 0:
                sign = 1
            else:
                sign = -1
                key_factor *= -1
            mult_factor = [0]*6
            mult_factor[key_factor - 1] = sign
            current_pose = self.move_group.get_current_pose().pose

            new_pos = current_pose.position
            new_pos.x += mult_factor[0] * position_step
            new_pos.y += mult_factor[1] * position_step
            new_pos.z += mult_factor[2] * position_step

            orient = current_pose.orientation
            roll, pitch, yaw = self.quat_to_eul([orient.x, orient.y, orient.z, orient.w])

            roll += mult_factor[3] * orientation_step
            pitch += mult_factor[4] * orientation_step
            yaw += mult_factor[5] * orientation_step

            new_orient = Quaternion(*self.eul_to_quat(roll, pitch, yaw))

            goal_pose = Pose(new_pos, new_orient)

            self.move_group.set_pose_target(goal_pose)
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            print 'Position'
            print '\tx: '+ str(new_pos.x)
            print '\ty: '+ str(new_pos.y)
            print '\tz: '+ str(new_pos.z)
            print 'Orientation'
            print '\troll: '+ str(roll)
            print '\tpitch: '+ str(pitch)
            print '\tyaw: '+ str(yaw)
            print '\n'


class WholeManip:
    def __init__(self, argv):
        moveit_commander.roscpp_initialize(argv)
        self.robot = moveit_commander.RobotCommander()


if __name__ == '__main__':
    rospy.init_node('pick_and_place')
    WholeManip(sys.argv)
    arm = ArmManip();
    arm.man_control();
