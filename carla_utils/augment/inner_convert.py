
import numpy as np
import carla

from .state import State
from .waypoint import Waypoint

class InnerConvert(object):
    @staticmethod
    def WaypointToState(waypoint, k=0.0, v=0.0):
        '''
            Args:
                waypoint: carla_utils.augment.Waypoint
        '''
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
    
    @staticmethod
    def CarlaTransformToState(frame_id, time_stamp, transform, **kwargs):
        location, rotation = transform.location, transform.rotation
        x, y, z = location.x, location.y, location.z
        theta = np.deg2rad(rotation.yaw)
        kwargs.update({'x':x, 'y':y, 'z':z, 'theta':theta})
        return State(frame_id, time_stamp, **kwargs)
