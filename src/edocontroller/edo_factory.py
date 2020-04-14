#!/usr/bin/env python

from edocontroller.robot_interface.edo_dummy import EdoDummy
from edocontroller.robot_interface.moveit_controller import MoveitController
from edocontroller.robot_interface.moveit_simulator import MoveitSimulator

DUMMY = "dummy"
SIMULATION = "simulation"
REAL_ROBOT = "connexion"

class EdoFactory:
    """This class is used to instantiate a controller"""

    def create_robot_model(self, connexion_done):
        if connexion_done == REAL_ROBOT:
            return MoveitController()
        elif connexion_done == SIMULATION:
            return MoveitSimulator()
        else:
            return EdoDummy()
