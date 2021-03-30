#!/usr/bin/env python

from math import pi

import rospy

from edocontroller import EdoFactory, SIMULATION

if __name__ == "__main__":
    rospy.init_node("simple joint control", anonymous=True)

    # initialize the controller wanted, here the one used along with the simulation
    controller = EdoFactory().create_robot_model(SIMULATION)

    # go to joints values wanted
    controller.go_to_joint(0, 0, 0, 0, pi/4, 0)