import carla

import time
import numpy as np

from .agent_base import BaseAgent
from agents.navigation.agent import Agent, AgentState

from ..augment import vector3DNorm
from ..world_map import draw_arrow, get_leading_vehicle_unsafe


class IDMAgent(BaseAgent, Agent):
    def __init__(self, config, client, world, town_map, vehicle, sensors_master, global_path=None):
        BaseAgent.__init__(self, config, client, world, town_map, vehicle, sensors_master, global_path)
        Agent.__init__(self, vehicle)

        self.leading_range = 50.0
        assert self.leading_range < self.distance_range

        self.desired_speed = self.max_velocity
        self.time_gap = 1.0
        self.min_gap = 2.0
        self.delta = 4.0
        self.acceleration = 1.0
        self.comfortable_deceleration = 1.5
    

    def get_target_v(self, reference):
        self._state = AgentState.NAVIGATING

        actor_list = self.world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        '''get traffic light'''
        light_state, traffic_light = self._is_light_red(lights_list)
        if light_state: self._state = AgentState.BLOCKED_RED_LIGHT

        '''get leading vehicle'''
        current_transform = self.vehicle.get_transform()
        reference_waypoints, remaining_distance = self.global_path.remaining_waypoints(current_transform)
        if remaining_distance < self.leading_range:
            self.reset_route()
            reference_waypoints, remaining_distance = self.global_path.remaining_waypoints(current_transform)

        vehicle, distance = get_leading_vehicle_unsafe(self.vehicle, vehicle_list, reference_waypoints, max_distance=self.leading_range)
        if vehicle is not None:
            draw_arrow(self.world, vehicle.get_transform(), life_time=0.1)
            self._state = AgentState.BLOCKED_BY_VEHICLE

        if self._state == AgentState.NAVIGATING:
            target_v = self.max_velocity
        elif self._state == AgentState.BLOCKED_RED_LIGHT:
            target_v = -1.0
        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            target_v = self.intelligent_driver_model(vehicle, distance)
        else: raise NotImplementedError
        return target_v


    def intelligent_driver_model(self, leading_vehicle, leading_distance):
        distance_c2c = leading_distance
        length_two_half = leading_vehicle.bounding_box.extent.x + self.vehicle.bounding_box.extent.x
        distance_b2b = distance_c2c - length_two_half - 0.3   # bumper-to-bumper distance
        distance_b2b_valid = max(0.001, distance_b2b)

        leading_v = vector3DNorm(leading_vehicle.get_velocity())    ## !warning
        current_v = self.get_current_v()
        delta_v = current_v - leading_v
        
        s = current_v*(self.time_gap+delta_v/(2*np.sqrt(self.acceleration*self.comfortable_deceleration)))
        distance_desired = self.min_gap + max(0, s)

        v_rational = (current_v / self.desired_speed)**self.delta
        s_rational = (distance_desired / distance_b2b_valid) ** 2
        acceleration_target = self.acceleration * (1 - v_rational - s_rational)
        target_v = current_v + acceleration_target / self.decision_frequency


        # print(distance_b2b, round(leading_v, 4), (target_v, current_v), s_rational)

        a = (distance_b2b, (self.min_gap + current_v*self.time_gap) / (1-(current_v/self.desired_speed)**self.delta)*0.5)
        print(a, (target_v, current_v), (acceleration_target, 1-v_rational, s_rational) )
        print()



        return target_v
    
