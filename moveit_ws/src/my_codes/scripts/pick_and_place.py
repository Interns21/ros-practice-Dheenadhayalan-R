#! /usr/bin/env python

import sys, rospy, tf, moveit_commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi


class ArmManip:
    """
    Arm manipulator
    """
    def __init__(self):
        self.eul_to_quat = tf.transformations.quaternion_from_euler
        self.quat_to_eul = tf.transformations.euler_from_quaternion
        self.group_name = 'panda_arm'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def move_to(self,point=None,orient=None):
        """
            Move to the given position
        """
        goal_pose = self.move_group.get_current_pose().pose  # current position
        if point is not None:  # set coordinates
            current_pose.position.x = point.x
            current_pose.position.y = point.y
            current_pose.position.z = point.z
        if orient is not None:  # yet to debug
            current_pose.orientation.x = orient.x
            current_pose.orientation.y = orient.y
            current_pose.orientation.z = orient.z
            current_pose.orientation.w = orient.w

        self.move_group.set_pose_target(goal_pose)  # set target
        self.move_group.go(wait=True)  # execute
        move_group.stop()  # stop residual movement

    def man_control(self):
        """
        Teleoperation function
        """
        control_keys = {'q': 1, 'w': -1,  # denotes control keys with position of change and sign of change
        'e': 2, 'r': -2,
        't': 3, 'y': -3,
        'a': 4, 's': -4,
        'd': 5, 'f': -5,
        'g': 6, 'h': -6}
        key_sub = rospy.Subscriber('keys', String, self.man_control_callback, control_keys)
        print 'Start pressing keys in key publisher'
        rospy.spin()

    def man_control_callback(self, key, control_keys):
        """
        callback for key press
        """
        position_step = 0.05  # step to move on position
        orientation_step = pi/20  # steps to move on orientation
        if key.data[0] in control_keys:
            key_factor = control_keys[key.data[0]]
            if key_factor > 0:
                sign = 1
            else:
                sign = -1
                key_factor *= -1
            mult_factor = [0]*6
            mult_factor[key_factor - 1] = sign  # obtained multiplication factor for x, y, z, roll, pitch, yaw
            current_pose = self.move_group.get_current_pose().pose

            new_pos = current_pose.position
            new_pos.x += mult_factor[0] * position_step  #
            new_pos.y += mult_factor[1] * position_step  # calculating new position
            new_pos.z += mult_factor[2] * position_step  #

            orient = current_pose.orientation
            roll, pitch, yaw = self.quat_to_eul([orient.x, orient.y, orient.z, orient.w])

            roll += mult_factor[3] * orientation_step   #
            pitch += mult_factor[4] * orientation_step  # calculating new orientation
            yaw += mult_factor[5] * orientation_step    #

            new_orient = Quaternion(*self.eul_to_quat(roll, pitch, yaw))

            goal_pose = Pose(new_pos, new_orient)  # obtaining goal pose

            self.move_group.set_pose_target(goal_pose)
            self.move_group.go(wait=True)  # reach the goal
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
    """
    Initialize and control arm and finger
    """
    def __init__(self, argv):
        moveit_commander.roscpp_initialize(argv)
        self.robot = moveit_commander.RobotCommander()


if __name__ == '__main__':
    rospy.init_node('pick_and_place')
    WholeManip(sys.argv)
    arm = ArmManip();
    arm.man_control();
