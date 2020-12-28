import carla

import time
import numpy as np

from ..augment import error_state


class Controller(object):
    def __init__(self, config, frequency):
        '''parameter'''
        wheelbase = config.get('wheelbase', 2.875)
        max_t = config.get('max_throttle', 1.0)
        max_b = config.get('max_brake', 1.0)
        max_steer = config.get('max_steer', 1.0)

        Kp, Ki, Kd = 1.00, 0.01, 0.05
        self.v_param = (Kp, Ki, Kd)
        k_theta, k_e = 0.2, 0.05
        self.w_param = (k_theta, k_e)

        self.v_controller = PID(1.0/frequency, -max_b, max_t)
        self.w_controller = RWPF(wheelbase, max_steer)


    def run_step(self, current_state, target_state):
        target_state.v -= 0.01
        throttle, brake = self.v_controller.run_step(current_state, target_state, self.v_param)
        steer = self.w_controller.run_step(current_state, target_state, self.w_param)
        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)



class PID(object):
    def __init__(self, dt, min_a, max_a):
        self.dt = dt
        self.min_a, self.max_a = min_a, max_a
        self.last_error = 0
        self.sum_error = 0

    def run_step(self, current_state, target_state, param):
        Kp, Ki, Kd = param[0], param[1], param[2]

        v_current = current_state.v
        v_target = target_state.v
        error = v_target - v_current

        acceleration = Kp * error
        acceleration += Ki * self.sum_error * self.dt
        acceleration += Kd * (error - self.last_error) / self.dt

        self.last_error = error
        self.sum_error += error
        '''eliminate drift'''
        if abs(self.sum_error) > 10:
            self.sum_error = 0.0

        throttle = np.clip(acceleration, 0, self.max_a)
        brake = -np.clip(acceleration, self.min_a, 0)
        return throttle, brake


class RWPF(object):
    """
        Paper: A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles
    """
    def __init__(self, wheelbase, max_steer):
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.curvature_factor = 1.0
        self.alpha = 1.8

    def run_step(self, current_state, target_state, param):
        k_theta, k_e = param[0], param[1]

        longitudinal_e, lateral_e, theta_e = error_state(current_state, target_state)

        vr, kr = target_state.v, target_state.k

        c1 = (kr*self.curvature_factor) *np.cos(theta_e)
        c2 = - k_theta *theta_e
        c3 = (k_e*np.exp(-theta_e**2/self.alpha))*lateral_e
        curvature = c1 + c2 + c3
        steer = np.arctan(curvature * self.wheelbase)
        return steer
