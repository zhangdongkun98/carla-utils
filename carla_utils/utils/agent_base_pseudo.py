import carla

import time
import numpy as np

from .agent_base import BaseAgent
from ..augment import InnerConvert


class BaseAgentPseudo(BaseAgent):
    def __init__(self, config, client, world, town_map, vehicle, sensors_master, global_path=None, fast=False):
        super(BaseAgentPseudo, self).__init__(config, client, world, town_map, vehicle, sensors_master, global_path)

        self.distance_range = 100
        self.sampling_resolution = 0.1

        self.pseudo, self.fast = True, fast
        self._current_transform = None
        self._current_v = 0.0
        self.dt = 1.0 / self.control_frequency
    

    def run_step(self):
        """
            Note: if carla runs in synchronous mode, then needs world.tick() after this method.
        
        Args:
        
        Returns:
            
        """
        
        self.tick_transform()
        target_v = self.get_target_v()
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            self.next_transform(target_v)
            if not self.fast: self.clock.tick_end()
        self.vehicle.set_transform(self._current_transform)
        return
    
    def get_transform(self):
        return self._current_transform
    def get_current_v(self):
        return self._current_v
    def tick_transform(self):
        self._current_transform = self.vehicle.get_transform()

    def next_transform(self, target_v):
        if self.global_path.reached(5.0):
            if self.random_walk: print('reset route!'); self.reset_route()
            else: print('goal reached!'); target_v = 0.0

        current_transform = self.get_transform()
        current_v = self.get_current_v()

        acceleration = np.clip((target_v-current_v)/self.dt, self.min_acceleration, self.max_acceleration)
        distance = np.clip(current_v * self.dt + 0.5 * acceleration * self.dt**2, 0, None)
        velocity = np.clip(current_v + acceleration * self.dt, 0, None)
        next_transform = self.global_path.next_waypoint(current_transform, distance).transform

        self._current_transform = next_transform
        self._current_v = velocity
        return
    