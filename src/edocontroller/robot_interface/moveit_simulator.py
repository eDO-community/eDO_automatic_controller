#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from edocontroller.robot_interface.moveit_controller import MoveitController


class MoveitSimulator(MoveitController):
    def __init__(self):
        MoveitController.__init__(self)
        self.channel_gripper = rospy.Publisher("/edo/set_gripper_span", Float32, queue_size=10)

    def open_gripper(self):
        rate = rospy.Rate(10)
        for i in range(4):
            self.channel_gripper.publish(0.15)
            rate.sleep()

    def close_gripper(self):
        rate = rospy.Rate(5)
        for i in range(3):
            self.channel_gripper.publish(0.05)
            rate.sleep()

    def _correct_pose_goal(self, pose_goal_geometry):
        # no correction done in simulation
        # because correction is hardcoded, not redefining this function makes the code crash
        return pose_goal_geometry

    def _prevent_collision_with_ground(self, pose_goal_in_moveit):
        # no need to prevent collision in simulation
        # because protection is hardcoded, not redefining this function makes the code crash
        return pose_goal_in_moveit
