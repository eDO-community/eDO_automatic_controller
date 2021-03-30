#!/usr/bin/env python3

import rospy


class EdoMonitor:
    """ NOT IMPLEMENTED
    This is a monitor of the edo robot,
    It provides information about edo at all times, and allows the robot to react in case of
    unexpected situation"""

    def __init__(self):
        self.current_state = 0
        self.collision_found = False

    def is_braked(self):
        pass
