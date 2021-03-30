#!/usr/bin/env python

import numpy as np
import quaternion

from .geometry_transformer import GeometryTransformer


class VectorQuaternion(object):
    """This class represents a 3D vector as a quaternion
    It allows to easily apply rotation on 3D vector
    It also implements 3D vector operation such as dot product or cross product
    """
    def __init__(self, vector):
        if isinstance(vector, quaternion.quaternion):
            # The quaternion given should represent a point, so w should be equal to 0
            # in practice, operations can create a very low value for w, so we put it to 0 in this case
            if abs(vector.w) < 10**(-8):
                vector.w = 0
            else:
                raise ValueError("quaternion should not have a value as w")
            self.vector = vector
        elif isinstance(vector, np.ndarray):
            self.vector = np.quaternion(0, vector[0], vector[1], vector[2])
        else:
            raise NotImplementedError("constructor not implemented with this type : " + str(vector.__class__))

    @property
    def x(self):
        return self.vector.x

    @property
    def y(self):
        return self.vector.y

    @property
    def z(self):
        print("getter")
        return self.vector.z

    @x.setter
    def x(self, x):
        self.vector.x = x

    @y.setter
    def y(self, y):
        self.vector.y = y

    @z.setter
    def z(self, z):
        self.vector.z = z

    def quaternion(self):
        return self.vector

    def np_array(self):
        return GeometryTransformer.np_array_from_quat(self.vector)

    def __repr__(self):
        return "VectorQuaternion ( " + self.vector.__repr__() + " )"

    def __eq__(self, other):
        if isinstance(other, VectorQuaternion):
            return other.vector == self.vector
        else:
            return False

    def __ne__(self, other):
        return not self == other

    def __add__(self, other):
        if isinstance(other, VectorQuaternion):
            return VectorQuaternion(self.vector + other.vector)
        elif isinstance(other, quaternion.quaternion):
            return VectorQuaternion(self.vector + other)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def __sub__(self, other):
        if isinstance(other, VectorQuaternion):
            return VectorQuaternion(self.vector - other.vector)
        elif isinstance(other, quaternion.quaternion):
            return VectorQuaternion(self.vector - other)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def __rmul__(self, other):
        if type(other) == int or type(other) == float:
            return VectorQuaternion(other*self.vector)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def __mul__(self, other):
        if type(other) == int or type(other) == float:
            return VectorQuaternion(other*self.vector)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def __imul__(self, other):
        if type(other) == int or type(other) == float:
            return VectorQuaternion(other*self.vector)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def __div__(self, other):
        if type(other) == int or type(other) == float:
            return VectorQuaternion(self.vector/other)
        else:
            raise NotImplementedError("operation not implemented with this type : " +str(other.__class__))

    def __neg__(self):
        return VectorQuaternion(-self.vector)

    def dot(self, other):
        if isinstance(other, VectorQuaternion):
            vector_array = GeometryTransformer.np_array_from_quat(self.vector)
            other_vector_array = GeometryTransformer.np_array_from_quat(other.vector)
            return vector_array.dot(other_vector_array)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def cross(self, other):
        if isinstance(other, VectorQuaternion):
            vector_array = GeometryTransformer.np_array_from_quat(self.vector)
            other_vector_array = GeometryTransformer.np_array_from_quat(other.vector)
            return VectorQuaternion(np.cross(vector_array, other_vector_array))
        else:
            raise NotImplementedError("operation not implemented with this type : " + other)

    def dot_matrix(self, other):
        if isinstance(other, VectorQuaternion):
            new_vector = VectorQuaternion(np.array([self.x*other.x, self.y*other.y, self.z*other.z]))
            return new_vector
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(other.__class__))

    def cross_normalized(self, other):
        array = self.cross(other)
        return array.normalized()

    def norm(self):
        return self.vector.norm()

    def normalized(self):
        return VectorQuaternion(self.vector.normalized())

    def apply_rotation_to_vector(self, rotation):
        rotation = rotation.normalized()
        new_point = rotation*self.vector/rotation
        return VectorQuaternion(new_point)

    def apply_rotation_and_norm_vector(self, rotation):
        """"rotation must be a quaternion"""
        if isinstance(rotation, quaternion.quaternion):
            rotation = rotation.normalized()
            new_point = rotation*self.vector/rotation
            new_point = new_point.normalized()
            # put w at 0 if low value appear because of error precision
            if (new_point.w < 10**(-8)):
                new_point.w = 0
                new_point.normalized()
            return VectorQuaternion(new_point)
        else:
            raise NotImplementedError("operation not implemented with this type : " + str(rotation.__class__))
