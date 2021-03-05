import carla

import numpy as np

from ..augment import GlobalPath, InnerConvert, vector3DNorm
from .agent_abc import AgentABC
from .vehicle_model import RealModel, BicycleModel2D


class BaseAgent(AgentABC):
    def __init__(self, config, world, town_map, vehicle, sensors_master, global_path: GlobalPath):
        """
        
        
        Args:
            config: contains decision_frequency, control_frequency,
                    max_velocity, max_acceleration, min_acceleration,
                    max_throttle, max_brake, max_steer
        
        Returns:
            None
        """

        super(BaseAgent, self).__init__(config, world, town_map, vehicle, sensors_master, global_path)

        self.pseudo = False

        self.vehicle_model = RealModel()
        self.run_step = self._run_step_real
        return


    def get_target(self, reference):
        return self.max_velocity
    
    def get_transform(self):
        return self.vehicle.get_transform()
    def get_current_v(self):
        return vector3DNorm(self.vehicle.get_velocity())
    def get_state(self):
        current_transform = self.get_transform()
        current_v = self.get_current_v()
        return InnerConvert.CarlaTransformToState(None, None, current_transform, v=current_v)


    def get_control(self, target):
        if self.goal_reached(0.0):
            self.extend_route(); #print('[BaseAgent] extend route!')

        target = self.max_velocity  ### TODO delete

        current_transform = self.get_transform()
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)
        # draw_arrow(self.world, target_waypoint.transform, life_time=0.1)

        current_v = self.get_current_v()
        current_state = InnerConvert.CarlaTransformToState(None, None, current_transform, v=current_v)
        target_state = InnerConvert.CarlaTransformToState(None, None, target_waypoint.transform, v=target, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control
    
    def forward(self, control):
        self.vehicle_model(self.vehicle, control)
    

class BaseAgentPseudo(BaseAgent, AgentABC):
    def __init__(self, config, world, town_map, vehicle, sensors_master, global_path):
        super(BaseAgentPseudo, self).__init__(config, world, town_map, vehicle, sensors_master, global_path)

        self.pseudo, self.fast = True, config.fast

        self._current_transform = self.vehicle.get_transform()
        self._current_v = 0.0

        self.vehicle_model = BicycleModel2D(self.dt, self.wheelbase)
        self.run_step = self._run_step_pseudo

    
    def get_transform(self):
        return self._current_transform
    def get_current_v(self):
        return self._current_v

    def tick_transform(self):
        self._current_transform = self.vehicle.get_transform()

    def forward(self, control):
        acceleration = self.controller.control_to_acceleration(control)
        action = (acceleration, control.steer)

        current_state = self.get_state()
        next_state = self.vehicle_model(current_state, action)
        x, y, theta, v = next_state.x, next_state.y, next_state.theta, next_state.v

        self._current_transform = carla.Transform(carla.Location(x=x, y=y), carla.Rotation(yaw=np.rad2deg(theta)))
        self._current_v = v
        return
        

