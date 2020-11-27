
import time
import numpy as np
import copy

from agents.navigation.local_planner import RoadOption
from .tools import error_transform, distance_waypoint

lane_change_set = set([RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT])
lane_keeping_set = set([RoadOption.VOID, RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT, RoadOption.LANEFOLLOW])


class GlobalPath(object):
    def __init__(self, frame_id, time_stamp, route, sampling_resolution):
        self.frame_id = frame_id
        self.time_stamp = time_stamp

        self.route = route
        self.carla_waypoints = list(np.array(route)[:,0])
        self.options = list(np.array(route)[:,1])
        self._destination = self.carla_waypoints[-1].transform
        self.sampling_resolution = sampling_resolution

        self.max_coverage = 0
        assert sampling_resolution <= 0.2


    def reset(self):
        '''
            Reset self.max_coverage to 0, since self.max_coverage aims to eliminate back-and-forth.
        '''
        self.max_coverage = 0


    def truncate(self, current_state, max_step):
        '''
            Args:
                current_state: any instance that contains attribute x, y, z
        '''
        raise NotImplementedError
        self._step_target(current_state, max_step)
        start, end = self.min_target, self.min_target+max_step
        reference_route = copy.copy(self.route[start:end])
        return reference_route
    

    def next_waypoint(self, current_transform, distance):
        '''
            TODO test on more scenes besides single lane, error is less than 10cm when sampling_resolution is 20cm
            Args:
                current_transform: carla.Transform
        '''
        self._step_coverage(current_transform)

        longitudinal_e, _, _ = error_transform(current_transform, self.carla_waypoints[self.max_coverage].transform)
        # assert longitudinal_e >= 0 or self.max_coverage == 0
        distance += longitudinal_e

        length = 0.0
        index = self.max_coverage
        for index in range(self.max_coverage, len(self)-1):
            length += distance_waypoint(self.carla_waypoints[index], self.carla_waypoints[index+1])
            if length >= distance:
                break

        waypoint_i, waypoint_ip = self.carla_waypoints[index], self.carla_waypoints[index+1]

        distance_previous = max(length-distance, 0.00001)
        distance_next = max(distance_waypoint(waypoint_i, waypoint_ip) -(length-distance), 0.00001)
        waypoints_ip = waypoint_ip.previous(distance_previous)
        waypoints_i = waypoint_i.next(distance_next)

        result, min_dist = None, float('inf')
        for next_waypoint in waypoints_i:
            for waypoint in waypoints_ip:
                dist = next_waypoint.transform.location.distance(waypoint.transform.location)
                if dist < min_dist:
                    min_dist = dist
                    result = next_waypoint
        return result
    

    def _step_coverage(self, current_transform):
        '''
            Args:
                current_transform: carla.Transform
        '''
        index = self.max_coverage
        for index in range(self.max_coverage, len(self)-2):
            longitudinal_e, _, _ = error_transform(current_transform, self.carla_waypoints[index+1].transform)
            if longitudinal_e < 0:
                break
        self.max_coverage = index



    def _calc_nearest_index(self, current_state, reference_waypoints):
        raise NotImplementedError
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


    def __len__(self):
        return len(self.route)

    @property
    def destination(self):
        return self._destination
    