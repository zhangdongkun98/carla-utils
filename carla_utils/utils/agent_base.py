import carla

import time

from ..augment import GlobalPath, InnerConvert, vector3DNorm
from .controller import Controller
from ..world_map import get_reference_route, draw_arrow


class BaseAgent(object):
    def __init__(self, config, world, town_map, vehicle, global_path=None):
        """
        
        
        Args:
            config: contains decision_frequency, control_frequency,
                    max_velocity, max_acceleration, min_acceleration,
                    max_throttle, max_brake, max_steer
        
        Returns:
            None
        """
        
        '''config, remind'''
        self.max_velocity = config.get('max_velocity', 8.34)
        self.max_acceleration = config.get('max_acceleration', 5.0)
        self.min_acceleration = config.get('min_acceleration', -5.0)
        self.max_throttle = config.get('max_throttle', 1.0)
        self.max_brake = config.get('max_brake', 1.0)
        self.max_steer = config.get('max_steer', 1.0)

        self.debug = config.get('debug', False)

        self.decision_frequency = config.decision_frequency
        self.control_frequency = config.control_frequency
        self.skip_num = self.control_frequency // self.decision_frequency
        assert self.control_frequency % self.decision_frequency == 0

        self.distance_range = 100
        self.sampling_resolution = 1

        self.controller = Controller(config, self.control_frequency)

        '''vehicle'''
        self.world, self.town_map, self.vehicle = world, town_map, vehicle

        self.global_path, self.random_walk = global_path, False
        if self.global_path is None: self.reset_route()
        elif self.debug: self.global_path.draw(self.world, life_time=15)
        return

    def reset_route(self):
        self.random_walk = True
        route = get_reference_route(self.town_map, self.vehicle, self.distance_range, self.sampling_resolution)
        self.global_path = GlobalPath(None, None, route)
        if self.debug: self.global_path.draw(self.world, life_time=15)

    def run_step(self):
        target_v = self._get_target_v()
        for _ in range(self.skip_num):
            tick1 = time.time()
            control = self.get_control(target_v)
            self.vehicle.apply_control(control)
            tick2 = time.time()
            sleep_time = 1/self.control_frequency-tick2+tick1
            time.sleep( max(0, sleep_time) )
        return
    
    def _get_target_v(self):
        return self.max_velocity

    def get_control(self, target_v):
        if self.global_path.reached(5.0):
            if self.random_walk: print('reset route!'); self.reset_route()
            else: print('goal reached!'); control = carla.VehicleControl(brake=1.0); target_v = 0.0

        current_transform = self.vehicle.get_transform()
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        if self.debug: draw_arrow(self.world, target_waypoint.transform, life_time=0.1)

        current_v = vector3DNorm(self.vehicle.get_velocity())
        current_state = InnerConvert.CarlaTransformToState(None, None, current_transform, v=current_v)
        target_state = InnerConvert.CarlaTransformToState(None, None, target_waypoint.transform, v=target_v, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control
