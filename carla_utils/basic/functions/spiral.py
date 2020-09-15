
import numpy as np
from scipy import integrate

class QuadraticSpiral(object):
    '''
        curve length start from 0
        p: parameter, np.array, column vector
        curvature_vector: np.array, row vector
    '''
    @staticmethod
    def x_local(s, p):
        return integrate.quad(QuadraticSpiral.cos_theta, 0,s, args=p)[0]
    @staticmethod
    def y_local(s, p):
        return integrate.quad(QuadraticSpiral.sin_theta, 0,s, args=p)[0]
    @staticmethod
    def cos_theta(s, p):
        return np.cos(QuadraticSpiral.theta(s, p))
    @staticmethod
    def sin_theta(s, p):
        return np.sin(QuadraticSpiral.theta(s, p))
    @staticmethod
    def theta(s, p):
        theta_vector = np.array([[s, s**2/2, s**3/3]])
        # print(theta_vector.shape, p.shape)
        return np.dot(theta_vector, p)[0][0]
    @staticmethod
    def curvature(s, p):
        curvature_vector = np.array([[1, s, s**2]])
        return np.dot(curvature_vector, p)[0][0]


class ConstantSpiral(object):
    @staticmethod
    def x_local(s, p):
        return integrate.quad(ConstantSpiral.cos_theta, 0,s, args=p)[0]
    @staticmethod
    def y_local(s, p):
        return integrate.quad(ConstantSpiral.sin_theta, 0,s, args=p)[0]
    @staticmethod
    def cos_theta(s, p):
        return np.cos(ConstantSpiral.theta(s, p))
    @staticmethod
    def sin_theta(s, p):
        return np.sin(ConstantSpiral.theta(s, p))
    @staticmethod
    def theta(s, p):
        return p[0][0] * s
    @staticmethod
    def curvature(s, p):
        return p[0][0]