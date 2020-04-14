#!/usr/bin/env python

from math import pi

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point

from edocontroller import EdoFactory, SIMULATION

if __name__ == "__main__":
    rospy.init_node("simple joint control", anonymous=True)

    # initialize the controller wanted, here the one used along with the simulation
    controller = EdoFactory().create_robot_model(SIMULATION)

    pose = Pose()
    # this orientation is the quaternion that will be used to do the rotation of the vector (0, 0, 1) in gazebo/rviz
    # the quaternion (0, 0, 0, 1) is the identity, meaning that the end effector will have the same orientation  and
    # direction as the vector (0, 0, 1)
    pose.orientation = Quaternion(0, 0, 0, 1)

    # this is the position of the point the robot should go to
    pose.position = Point(0.09, 0.09, 0.94)

    # the controller try to go to the pose specified.
    # Note that the orientation of the pose is automatically normalized
    controller.go_to_pose_goal(pose)