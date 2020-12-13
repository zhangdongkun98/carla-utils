import carla

import time

from .agent_base import BaseAgent
from agents.navigation.agent import Agent, AgentState


class NaiveAgent(BaseAgent, Agent):
    def __init__(self, config, client, world, town_map, vehicle, sensors_master, global_path=None):
        BaseAgent.__init__(self, config, client, world, town_map, vehicle, sensors_master, global_path)
        Agent.__init__(self, vehicle)
    

    def get_target_v(self, reference):
        hazard_detected = False

        actor_list = self.world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        light_state, traffic_light = self._is_light_red(lights_list)
        if light_state:
            self._state = AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True

        target_v = -1.0 if hazard_detected else self.max_velocity
        return target_v
