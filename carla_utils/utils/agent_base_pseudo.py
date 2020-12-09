import carla

import time
import numpy as np

from .agent_base import BaseAgent


class BaseAgentPseudo(BaseAgent):
    def __init__(self, config, world, town_map, vehicle, global_path=None, fast=False):
        super(BaseAgentPseudo, self).__init__(config, world, town_map, vehicle, global_path)

        self.distance_range = 100
        self.sampling_resolution = 0.1

        self.fast = fast
        self._current_transform = None
        self._current_v = 0.0
        self.dt = 1.0 / self.control_frequency
    

    def run_step(self, client):
        self._current_transform = self.vehicle.get_transform()
        tmp = self._current_transform
        target_v = self._get_target_v()
        dist = 0
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            dist += self.next_transform(target_v)
            if not self.fast: self.clock.tick_end()
        self.vehicle.set_transform(self._current_transform)
        print(dist, tmp.location.distance(self._current_transform.location))
        return
    
    def get_transform(self):
        return self._current_transform
    def get_current_v(self):
        return self._current_v

    def next_transform(self, target_v):
        if self.global_path.reached(5.0):
            if self.random_walk: print('reset route!'); self.reset_route()
            else: print('goal reached!'); control = carla.VehicleControl(brake=1.0); target_v = 0.0

        current_transform = self.get_transform()
        current_v = self.get_current_v()
        # if self.debug: draw_arrow(self.world, target_waypoint.transform, life_time=0.1)

        acceleration = np.clip((target_v-current_v)/self.dt, self.min_acceleration, self.max_acceleration)
        distance = current_v * self.dt + 0.5 * acceleration * self.dt**2
        velocity = current_v + acceleration * self.dt
        next_transform = self.global_path.next_waypoint(current_transform, distance).transform

        self._current_transform = next_transform
        # print(current_transform.location, next_transform.location, distance, velocity, acceleration)
        self._current_v = velocity
        return next_transform.location.distance(current_transform.location)
    