
import numpy as np


def vector3DEMul(vec1, vec2):
    '''
        vec1, vec2: carla.Vector3D
    '''
    return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z
def vector2DEMul(vec1, vec2):
    '''
        vec1, vec2: carla.Vector3D
    '''
    return vec1.x*vec2.x + vec1.y*vec2.y

def vector3DNorm(vec):
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2)
def vector2DNorm(vec):
    return np.sqrt(vec.x**2 + vec.y**2)


def vectorYawRad(vec):
    vec_norm = vector2DNorm(vec)
    if vec_norm < 0.001:
        return 0.0
    v = vec / vec_norm
    x, y = v.x, v.y
    yaw_rad = np.arctan2(y, x)
    return yaw_rad


def vector3DToArray(vec):
    return np.array([vec.x, vec.y, vec.z]).reshape(3,1)