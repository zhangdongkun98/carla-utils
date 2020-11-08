
import numpy as np
import carla

from .state import State

class InnerConvert(object):
    @staticmethod
    def WaypointToState(waypoint, k=0.0, v=0.0):
        state = State(
                waypoint.frame_id, waypoint.time_stamp,
                x=waypoint.x, y=waypoint.y, z=waypoint.z,
                theta=waypoint.theta, k=k, v=v
                )
        return state
    
    @staticmethod
    def StateToCarlaTransform(state, town_map):
        x, y, theta = state.x, state.y, state.theta
        location = carla.Location(x=x, y=y)
        transform = town_map.get_waypoint(location).transform
        transform.rotation.yaw = np.rad2deg(theta)
        return transform