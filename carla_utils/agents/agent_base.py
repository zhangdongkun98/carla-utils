import carla
DestroyActor = carla.command.DestroyActor

import time

from .controller import Controller
from ..augment import GlobalPath, InnerConvert, vector3DNorm
from ..world_map import get_reference_route, draw_arrow
from ..sensor import CarlaSensorListMaster
from ..system import Clock


class BaseAgent(object):
    def __init__(self, config, world, town_map, vehicle, sensors_master, global_path=None):
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
        self.min_acceleration = config.get('min_acceleration', -10.0)
        self.max_throttle = config.get('max_throttle', 1.0)
        self.max_brake = config.get('max_brake', 1.0)
        self.max_steer = config.get('max_steer', 1.0)

        self.debug = config.get('debug', False)
        self.pseudo = False

        self.decision_frequency = config.decision_frequency
        self.control_frequency = config.control_frequency
        self.skip_num = self.control_frequency // self.decision_frequency
        assert self.control_frequency % self.decision_frequency == 0

        self.distance_range = 100
        self.sampling_resolution = 1

        self.clock = Clock(self.control_frequency)
        self.controller = Controller(config, self.control_frequency)

        '''vehicle'''
        self.world, self.town_map, self.vehicle = world, town_map, vehicle
        self.sensors_master : CarlaSensorListMaster = sensors_master

        self.id = vehicle.id

        self.global_path, self.random_walk = global_path, False
        if self.global_path is None: self.reset_route()
        elif self.debug: self.global_path.draw(self.world, life_time=15)
        return

    def reset(self):
        if self.random_walk: return
        self.sensors_master.reset()
        self.global_path.reset()

    def reset_route(self):
        self.random_walk = True
        route = get_reference_route(self.town_map, self.vehicle, self.distance_range, self.sampling_resolution)
        self.global_path = GlobalPath(None, None, route)
        if self.debug: self.global_path.draw(self.world, life_time=15)
    
    def mode_random_walk(self): self.reset_route()

    def run_step(self, reference):
        target_v = self.get_target_v(reference)
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            control = self.get_control(target_v)
            self.vehicle.apply_control(control)
            self.clock.tick_end()
        return
    
    def get_target_v(self, reference):
        return self.max_velocity
    
    def get_transform(self):
        return self.vehicle.get_transform()
    def get_current_v(self):
        return vector3DNorm(self.vehicle.get_velocity())
    def get_state(self):
        current_transform = self.get_transform()
        current_v = self.get_current_v()
        return InnerConvert.CarlaTransformToState(None, None, current_transform, v=current_v)
    
    def goal_reached(self, preview_distance):
        return self.global_path.reached(preview_distance)

    def get_control(self, target_v):
        if self.goal_reached(0.0):
            if self.random_walk: self.reset_route(); #print('reset route!')
            # else: print('goal reached!')

        current_transform = self.get_transform()
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        if self.debug: draw_arrow(self.world, target_waypoint.transform, life_time=0.1)

        current_v = self.get_current_v()
        current_state = InnerConvert.CarlaTransformToState(None, None, current_transform, v=current_v)
        target_state = InnerConvert.CarlaTransformToState(None, None, target_waypoint.transform, v=target_v, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control
    

    def destroy(self):
        self.sensors_master.destroy()
        self.vehicle.destroy()
    
    def destroy_commands(self):
        cmds = self.sensors_master.destroy_commands()
        cmds.append(DestroyActor(self.vehicle))
        return cmds
    

    def check_collision(self):
        return self.sensors_master[('sensor.other.collision', 'default')].get_raw_data() is not None


        
