
import carla
import numpy as np
import copy

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from .. import basic
from ..augment import GlobalPath, vectorYawRad
from ..world_map import draw_waypoints, get_reference_route_wrt_waypoint


class AgentsRoutePlanner(object):
    def __init__(self, world, town_map, vehicle, config, distance_range):
        '''
        Args:
            hop_resolution: [m] distance between adjacent points
        '''
        self.world, self.town_map, self.vehicle = world, town_map, vehicle

        self.global_frame_id = config.ego_vehicle.global_frame_id
        self.vehicle_frame_id = config.ego_vehicle.vehicle_frame_id
        self.goal_tolerance = config.ego_vehicle.goal_tolerance
        self.sampling_resolution = config.ego_vehicle.sampling_resolution
        self.distance_range = distance_range
        self.max_step = round(self.distance_range / float(self.sampling_resolution)) + 1

        dao = GlobalRoutePlannerDAO(town_map, self.sampling_resolution)
        self.grp = GlobalRoutePlanner(dao)
        self.grp.setup()

        # self.destination, self.route_waypoints = None, None
        self.global_path = None


    def trace_route(self, time_stamp, destination):
        '''
        Args:
            destination: carla.Location
        '''
        route = self.grp.trace_route(self.vehicle.get_location(), destination)

        '''remove waypoint whose distance too small'''
        simplified_route = copy.copy(route)
        delete_index_list = []
        for i in range(len(route)-1):
            loc_old, loc_new = route[i][0].transform.location, route[i+1][0].transform.location
            if loc_old.distance(loc_new) < self.sampling_resolution * 0.75:
                delete_index_list.append(i+1)
        basic.list_del(simplified_route, delete_index_list)

        '''
        simplified_route_again = copy.copy(simplified_route)
        delete_index_list = []
        for i in range(len(simplified_route)-1):
            loc_old, loc_new = simplified_route[i][0].transform.location, simplified_route[i+1][0].transform.location
            theta1 = np.deg2rad(simplified_route[i][0].transform.rotation.yaw)
            theta2 = vectorYawRad(loc_new-loc_old)
            flag = abs(abs(basic.pi2pi(theta2-theta1))-np.pi/2) < 0.01
            if flag:
                delete_index_list.append(i+1)
        basic.list_del(simplified_route_again, delete_index_list)
        '''

        self.global_path = GlobalPath(self.global_frame_id, time_stamp, simplified_route, self.sampling_resolution)
        return self.global_path

    def is_goal_reached(self):
        if self.global_path is None:
            return True
        distance = self.vehicle.get_location().distance(self.global_path.destination)
        return distance < self.goal_tolerance

    def truncate_global_path(self, current_state):
        reference_route = self.global_path.truncate(current_state, self.max_step)
        '''compensate route when close to destination'''
        if len(reference_route) < self.max_step:
            destination = reference_route[-1][0]
            extended_length = self.max_step-len(reference_route)
            extended_route = get_reference_route_wrt_waypoint(destination, self.sampling_resolution, extended_length)
            reference_route.extend(extended_route)
        return reference_route

    def draw_global_path(self, life_time=500):
        draw_waypoints(self.world, self.global_path.carla_waypoints, life_time=life_time)
