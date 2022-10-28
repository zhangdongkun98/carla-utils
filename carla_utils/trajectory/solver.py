
import numpy as np

class QuinticPolynomialSolver(object):
    """
        NumPy version. Supoort batch.
    """
    def __init__(self):
        return


    def solve(self, p0, p0_d, p0_dd, p1, p1_d, p1_dd, T):
        A = np.stack([
            self.time_matrix(T*0),
            self.time_matrix_d(T*0),
            self.time_matrix_dd(T*0),
            self.time_matrix(T),
            self.time_matrix_d(T),
            self.time_matrix_dd(T),
        ], axis=-2)
        b = np.expand_dims(np.stack([p0, p0_d, p0_dd, p1, p1_d, p1_dd], axis=-1), axis=-1)

        solution = np.matmul(np.linalg.inv(A), b)
        return solution


    @classmethod
    def time_matrix(cls, times):
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([one, times, times**2, times**3, times**4, times**5], axis=-1)

    @classmethod
    def time_matrix_d(cls, times):
        zero = np.zeros_like(times, dtype=np.float32)
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([zero, one, 2*times, 3*times**2, 4*times**3, 5*times**4], axis=-1)

    @classmethod
    def time_matrix_dd(cls, times):
        zero = np.zeros_like(times, dtype=np.float32)
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([zero, zero, 2*one, 6*times, 12*times**2, 20*times**3], axis=-1)




class QuarticPolynomialSolver(object):
    """
        NumPy version. Supoort batch.
    """
    def __init__(self):
        return


    def solve(self, p0, p0_d, p0_dd, p1_d, p1_dd, T):
        A = np.stack([
            self.time_matrix(T*0),
            self.time_matrix_d(T*0),
            self.time_matrix_dd(T*0),
            self.time_matrix_d(T),
            self.time_matrix_dd(T),
        ], axis=-2)
        b = np.expand_dims(np.stack([p0, p0_d, p0_dd, p1_d, p1_dd], axis=-1), axis=-1)

        solution = np.matmul(np.linalg.inv(A), b)
        return solution


    @classmethod
    def time_matrix(cls, times):
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([one, times, times**2, times**3, times**4], axis=-1)

    @classmethod
    def time_matrix_d(cls, times):
        zero = np.zeros_like(times, dtype=np.float32)
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([zero, one, 2*times, 3*times**2, 4*times**3], axis=-1)

    @classmethod
    def time_matrix_dd(cls, times):
        zero = np.zeros_like(times, dtype=np.float32)
        one = np.ones_like(times, dtype=np.float32)
        return np.stack([zero, zero, 2*one, 6*times, 12*times**2], axis=-1)

