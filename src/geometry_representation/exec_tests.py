from vector_quaternion import VectorQuaternion

import numpy as np
import quaternion


if __name__ == "__main__":
    a = VectorQuaternion(np.quaternion(0, 1, 2, 3))
    b = VectorQuaternion(np.quaternion(0, 2, 1, 3))
    q = np.quaternion(1, 0, 0, 0)
    print(type(q))
    print(a * b)
    a*q
    q*a
