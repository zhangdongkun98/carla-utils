
import carla
import numpy as np
import copy
import time

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from .. import basic
from ..augment import GlobalPath, vectorYawRad
from ..world_map import draw_waypoints, get_reference_route_wrt_waypoint


class AgentsRoutePlanner(object):
    def __init__(self, world, town_map, config):
        '''
        Args:
            config: need to contain:
                config.global_frame_id
                config.vehicle_frame_id
                config.goal_tolerance
                config.sampling_resolution
                config.distance_range
        '''
        self.world, self.town_map = world, town_map

        self.global_frame_id = config.get('global_frame_id', 'odom')
        self.vehicle_frame_id = config.get('vehicle_frame_id', 'base_link')
        self.goal_tolerance = config.goal_tolerance
        self.sampling_resolution = config.sampling_resolution
        self.distance_range = config.distance_range
        self.max_step = round(self.distance_range / float(self.sampling_resolution)) + 1

        dao = GlobalRoutePlannerDAO(town_map, self.sampling_resolution)
        self.grp = GlobalRoutePlanner(dao)
        self.grp.setup()


    def trace_route(self, origin, destination, time_stamp=time.time()):
        '''
        Args:
            origin, destination: carla.Location
        '''
        route = self.grp.trace_route(origin, destination)

        '''remove waypoint whose distance too small'''
        simplified_route = copy.copy(route)
        delete_index_list = []
        for i in range(len(route)-1):
            loc_old, loc_new = route[i][0].transform.location, route[i+1][0].transform.location
            if loc_old.distance(loc_new) < self.sampling_resolution * 0.75:
                delete_index_list.append(i+1)
        basic.list_del(simplified_route, delete_index_list)

        return GlobalPath(self.global_frame_id, time_stamp, simplified_route, self.sampling_resolution)

    def is_goal_reached(self, current_state, global_path):
        distance = current_state.distance(global_path.destination)
        return distance < self.goal_tolerance

    def truncate_global_path(self, current_state, global_path):
        reference_route = global_path.truncate(current_state, self.max_step)
        '''compensate route when close to destination'''
        if len(reference_route) < self.max_step:
            destination = reference_route[-1][0]
            extended_length = self.max_step-len(reference_route)
            extended_route = get_reference_route_wrt_waypoint(destination, self.sampling_resolution, extended_length)
            reference_route.extend(extended_route)
        return reference_route

    def draw_global_path(self, life_time=500):
        draw_waypoints(self.world, self.global_path.carla_waypoints, life_time=life_time)
