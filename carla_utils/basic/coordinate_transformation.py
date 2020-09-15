
'''
    Introduction to Robotics
        coordinate A (source), B (target)
        all variable: represent B in A
'''

import numpy as np
from .tools import np_dot

class RotationMatrix(object):
    '''
        get rotation matrix from rpy
    '''
    @staticmethod
    def roll(roll):
        R = np.identity(3)
        R[1,1] = np.cos(roll)
        R[2,2] = np.cos(roll)
        R[2,1] = np.sin(roll)
        R[1,2] =-np.sin(roll)
        return R
    @staticmethod
    def pitch(pitch):
        R = np.identity(3)
        R[0,0] = np.cos(pitch)
        R[2,2] = np.cos(pitch)
        R[2,0] =-np.sin(pitch)
        R[0,2] = np.sin(pitch)
        return R
    @staticmethod
    def yaw(yaw):
        R = np.identity(3)
        R[0,0] = np.cos(yaw)
        R[1,1] = np.cos(yaw)
        R[1,0] = np.sin(yaw)
        R[0,1] =-np.sin(yaw)
        return R

    @staticmethod
    def ypr(roll, pitch, yaw):
        r, p, y = RotationMatrix.roll, RotationMatrix.pitch, RotationMatrix.yaw
        return np_dot(r(roll), p(pitch), y(yaw))
    @staticmethod
    def rpy(roll, pitch, yaw):
        r, p, y = RotationMatrix.roll, RotationMatrix.pitch, RotationMatrix.yaw
        return np_dot(y(yaw), p(pitch), r(roll))


def HomogeneousMatrix(*args):
    if len(args) == 1:
        xyzrpy = args[0]
        if len(xyzrpy) != 6:
            raise ValueError("Must supply 6 values to build transform")
        R = RotationMatrix.rpy(xyzrpy[3], xyzrpy[4], xyzrpy[5])
        t = np.array(xyzrpy[0:3]).reshape(3,1)
    elif len(args) == 2:
        R, t = args[0], args[1]
    return np.vstack((np.hstack((R, t)), np.array([0,0,0,1])))

def HomogeneousMatrixInverse(*args):
    if len(args) == 1:
        xyzrpy = args[0]
        if len(xyzrpy) != 6:
            raise ValueError("Must supply 6 values to build transform")
        R = RotationMatrix.rpy(xyzrpy[3], xyzrpy[4], xyzrpy[5])
        t = np.array(xyzrpy[0:3]).reshape(3,1)
    elif len(args) == 2:
        R, t = args[0], args[1]
    return np.vstack((np.hstack((R.T, -np.dot(R.T, t))), np.array([0,0,0,1])))

def HomogeneousMatrix2D(R, t):
    return np.vstack((np.hstack((R, t)), np.array([0,0,1])))


class RotationMatrixTranslationVector(object):
    @staticmethod
    def homogeneous_matrix(HTM):
        return HTM[:3,:3], HTM[:3,3:]



class Euler(object):
    MATRIX_MATCH_TOLERANCE = 1e-4
    @staticmethod
    def rotation_matrix(R):
        if R.shape != (3, 3):
            raise ValueError("rotation matrix must be 3x3")
        roll = np.arctan2(R[2,1], R[2,2])
        yaw = np.arctan2(R[1,0], R[0,0])
        denom = np.sqrt(R[0,0]**2 + R[1,0]**2)
        pitch_poss = [np.arctan2(-R[2,0], denom), np.arctan2(-R[2,0], -denom)]

        R_ref = RotationMatrix.rpy(roll, pitch_poss[0], yaw)
        if (R - R_ref).sum() < Euler.MATRIX_MATCH_TOLERANCE:
            return roll, pitch_poss[0], yaw
        else:
            R_ref = euler_to_so3((roll, pitch_poss[1], yaw))
            if (R - R_ref).sum() > Euler.MATRIX_MATCH_TOLERANCE:
                raise ValueError("Could not find valid pitch angle")
            return roll, pitch_poss[1], yaw
