import carla

import time
import numpy as np
import copy
import random

from agents.navigation.local_planner import RoadOption
from .tools import error_transform, distance_waypoint
from .. import basic

lane_change_set = set([RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT])
lane_keeping_set = set([RoadOption.VOID, RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT, RoadOption.LANEFOLLOW])


def calc_curvature_with_yaw_diff(x, y, yaw):
    dists = np.array([np.hypot(dx, dy) for dx, dy in zip(np.diff(x), np.diff(y))])
    d_yaw = basic.pi2pi(np.diff(yaw))
    curvatures = d_yaw / dists
    curvatures = np.concatenate([curvatures, [0.0]])
    return curvatures, dists


class GlobalPath(object):
    def __init__(self, frame_id, time_stamp, route):
        self.frame_id = frame_id
        self.time_stamp = time_stamp

        self.route = route
        self.carla_waypoints = list(np.array(route)[:,0])
        self.options = list(np.array(route)[:,1])
        self._destination = self.carla_waypoints[-1].transform

        x = [i.transform.location.x for i in self.carla_waypoints]
        y = [i.transform.location.y for i in self.carla_waypoints]
        theta = [np.deg2rad(i.transform.rotation.yaw) for i in self.carla_waypoints]
        self.curvatures, self.distances = calc_curvature_with_yaw_diff(x, y, theta)
        self.sampling_resolution = np.average(self.distances)

        self._max_coverage = 0
        # assert self.sampling_resolution <= 0.2


    def __len__(self):
        return len(self.route)

    def reset(self):
        '''
            Reset self._max_coverage to 0, since self._max_coverage aims to eliminate back-and-forth.
        '''
        self._max_coverage = 0
    
    @property
    def destination(self):
        return self._destination
    @property
    def max_coverage(self):
        return self._max_coverage
    
    def reached(self, preview_distance=0):
        preview_distance = max(0, preview_distance)
        preview_index = max(preview_distance // self.sampling_resolution + 1, 0)
        return self._max_coverage >= len(self)-1 - preview_index

    
    def draw(self, world, size=0.1, color=(0,255,0), life_time=10):
        for waypoint in self.carla_waypoints:
            world.debug.draw_point(waypoint.transform.location, size=size, color=carla.Color(*color), life_time=life_time)
        return


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

        longitudinal_e, _, _ = error_transform(current_transform, self.carla_waypoints[self._max_coverage].transform)
        # assert longitudinal_e >= 0 or self._max_coverage == 0
        distance += longitudinal_e

        length = 0.0
        index = self._max_coverage
        for index in range(self._max_coverage, len(self)-1):
            length += distance_waypoint(self.carla_waypoints[index], self.carla_waypoints[index+1])
            if length >= distance:
                break
        
        if index == len(self)-1: return random.choice(self.carla_waypoints[index].next(distance))

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
    
    def target_waypoint(self, current_transform):
        self._step_coverage(current_transform)
        index = min(len(self)-1, self._max_coverage+1)
        return self.carla_waypoints[index], self.curvatures[index]
    
    def remaining_waypoints(self, current_transform):
        self._step_coverage(current_transform)
        return self.carla_waypoints[self._max_coverage:], sum(self.distances[self._max_coverage:])
    

    def _step_coverage(self, current_transform):
        '''
            Args:
                current_transform: carla.Transform
        '''
        index = self._max_coverage
        for index in range(self._max_coverage, len(self)):
            longitudinal_e, _, _ = error_transform(current_transform, self.carla_waypoints[min(len(self)-1, index+1)].transform)
            if longitudinal_e < 0:
                break
        self._max_coverage = index


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

    