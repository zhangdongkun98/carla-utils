import carla
DestroyActor = carla.command.DestroyActor

from abc import ABC, abstractmethod, abstractproperty

from ..augment import GlobalPath
from ..sensor import CarlaSensorListMaster
from ..world_map import get_reference_route_wrt_waypoint
from ..system import Clock
from .controller import Controller
from .tools import vehicle_wheelbase


class AgentABC(ABC):
    def __init__(self, config, world, town_map, vehicle, sensors_master, global_path: GlobalPath):
        """
        
        
        Args:
            config: contains decision_frequency, control_frequency,
                    max_velocity, max_acceleration, min_acceleration,
                    max_throttle, max_brake, max_steer
        
        Returns:
            None
        """
        
        '''config'''
        self.pseudo = None

        self.decision_frequency = config.decision_frequency
        self.control_frequency = config.control_frequency
        self.skip_num = self.control_frequency // self.decision_frequency
        assert self.control_frequency % self.decision_frequency == 0

        self.distance_range = float(config.perception_range)

        self.clock = Clock(self.control_frequency)

        '''vehicle'''
        self.world, self.town_map, self.vehicle = world, town_map, vehicle
        self.sensors_master: CarlaSensorListMaster = sensors_master
        self.global_path: GlobalPath = global_path

        self.id = vehicle.id
        self.decision_dt, self.control_dt = 1.0/self.decision_frequency, 1.0/self.control_frequency
        self.wheelbase = vehicle_wheelbase(self.vehicle)
        self.controller = Controller(config, self.control_dt, self.wheelbase)
        self.max_velocity = float(config.get('max_velocity', 8.34))
        self.max_acceleration = self.controller.max_acceleration
        self.min_acceleration = self.controller.min_acceleration
        self.max_throttle = self.controller.max_throttle
        self.max_brake = self.controller.max_brake
        self.max_steer = self.controller.max_steer

        self.vehicle_model = None
        return


    def extend_route(self):
        waypoint = self.global_path.carla_waypoints[-1]
        sr = self.global_path.sampling_resolution
        route = get_reference_route_wrt_waypoint(waypoint, sr, round(3*self.distance_range / sr))
        self.global_path.extend(route[1:])
        # self.global_path.draw(self.world, life_time=0)

    def goal_reached(self, preview_distance):
        return self.global_path.reached(preview_distance)

    def destroy(self):
        self.sensors_master.destroy()
        self.vehicle.destroy()
    
    def destroy_commands(self):
        cmds = self.sensors_master.destroy_commands()
        cmds.append(DestroyActor(self.vehicle))
        return cmds
    

    def check_collision(self):
        return self.sensors_master[('sensor.other.collision', 'default')].get_raw_data() is not None



    def _run_step_real(self, reference):
        target = self.get_target(reference)
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            if self.goal_reached(5.0): self.extend_route()
            control = self.get_control(target)
            self.forward(control)
            self.clock.tick_end()
        return

    def _run_step_pseudo(self, reference):
        """
            Note: if carla runs in synchronous mode, then needs world.tick() after this method.
        
        Args:
        
        Returns:
            
        """
        
        self.tick_transform()
        target = self.get_target(reference)
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            if self.goal_reached(5.0): self.extend_route()
            control = self.get_control(target)
            self.forward(control)
            if not self.fast: self.clock.tick_end()
        self.vehicle.set_transform(self._current_transform)
        return

