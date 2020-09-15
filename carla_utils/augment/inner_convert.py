
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