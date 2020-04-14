#!/usr/bin/env python

from math import pi

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point

from edocontroller import EdoFactory, SIMULATION

if __name__ == "__main__":
    rospy.init_node("simple joint control", anonymous=True)

    # initialize the controller wanted, here the one used along with the simulation
    controller = EdoFactory().create_robot_model(SIMULATION)

    # this returns the current pose. The return type is a GeometryPose object
    geometry_pose = controller.get_current_pose()

    # change position to move down
    geometry_pose.position.z -= 0.1

    # the controller try to go to the pose specified.
    # the go to pose_goal function accepts Pose and GeometryPose variable
    controller.go_to_pose_goal(geometry_pose)