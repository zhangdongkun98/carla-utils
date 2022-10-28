from carla_utils import carla
DestroyActor = carla.command.DestroyActor

from ..sensor import CarlaSensorListMaster
from ..world_map import Role
from .agent_base import BaseAgent


class DefaultAgent(BaseAgent):
    def get_target(self, reference):
        return 0.0

    def get_control(self, target):
        return carla.VehicleControl()
    
    def forward(self, control):
        return


    def extend_route(self):
        return

    def goal_reached(self, preview_distance):
        return False



    def _run_step_fast(self, reference):
        raise RuntimeError('[BaseObstacle] Forbidden')


    def _run_step_real(self, reference):
        raise RuntimeError('[BaseObstacle] Forbidden')

    def _run_control(self, stop):
        raise RuntimeError('[BaseObstacle] Forbidden')
