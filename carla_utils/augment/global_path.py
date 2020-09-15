
import time
import numpy as np
import copy

from agents.navigation.local_planner import RoadOption

lane_change_set = set([RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT])
lane_keeping_set = set([RoadOption.VOID, RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT, RoadOption.LANEFOLLOW])


class GlobalPath(object):
    def __init__(self, frame_id, time_stamp, route, sampling_resolution):
        self.frame_id = frame_id
        self.time_stamp = time_stamp

        self.route = route
        self.carla_waypoints = list(np.array(route)[:,0])
        self.options = list(np.array(route)[:,1])
        self._destination = self.carla_waypoints[-1].transform.location
        self.sampling_resolution = sampling_resolution

        self.min_target = 0


    def truncate(self, current_state, max_step):
        start_p, end_p = self.min_target, self.min_target+max_step
        reference_waypoints_p = self.carla_waypoints[start_p:end_p]
        local_target, _ = self._calc_nearest_index(current_state, reference_waypoints_p)
        self.min_target += local_target
        start, end = self.min_target, self.min_target+max_step
        reference_route = copy.copy(self.route[start:end])
        return reference_route


    def _calc_nearest_index(self, current_state, reference_waypoints):
        ind, mind = -1, float('inf')
        x, y, z = current_state.x, current_state.y, current_state.z
        for index, waypoint in enumerate(reference_waypoints):
            dx, dy = x - waypoint.transform.location.x, y - waypoint.transform.location.y
            dz = z - waypoint.transform.location.z
            d = dx**2 + dy**2 + dz**2
            if d < mind:
                mind = d
                ind = index
        return ind, np.sqrt(mind)


    @property
    def destination(self):
        return self._destination
    