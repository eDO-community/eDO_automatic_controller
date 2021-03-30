#!/usr/bin/env python

import numpy as np
import quaternion

from .vector_quaternion import VectorQuaternion

class GeometryPose:
    """Representation of pose message using VectorQuaternion and np quaternion"""
    def __init__(self, position, orientation):
        # two VectorQuaternion should be given, and the one with position should have a 0 as w
        if isinstance(position, quaternion.quaternion) or isinstance(position, np.ndarray):
            self.position = VectorQuaternion(position)
        elif isinstance(position, VectorQuaternion):
            self.position = position
        else:
            raise NotImplementedError("position of type : " + str(position.__class__.__name__) +
                                      " does not permit initialisation of class")

        if isinstance(orientation, quaternion.quaternion):
            self.orientation = orientation
            self.orientation = self.orientation.normalized()
        else:
            raise NotImplementedError("orientation of type : " + str(orientation.__class__.__name__) +
                                      " does not permit initialisation of class")

    def __repr__(self):
        return "[Position : np_" + repr(self.position) + "\nOrientation : np_" + repr(self.orientation) + "]"
