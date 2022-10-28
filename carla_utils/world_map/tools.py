
from carla_utils import carla, navigation

import numpy as np
import random
import os, glob
from os.path import join

from ..basic import Data
from ..system import get_carla_version, new_versions


def get_spawn_transform(core, spawn_point, height=0.1):
    '''
        only use x,y of spawn_point
    '''
    town_map = core.town_map
    x, y = spawn_point.x, spawn_point.y
    spawn_transform = town_map.get_waypoint(carla.Location(x=x, y=y)).transform
    spawn_transform.location.z += height
    return spawn_transform



def remove_traffic_light(vehicle, time=10000.0):
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            traffic_light.set_state(carla.TrafficLightState.Green)
            traffic_light.set_green_time(time)
    return


def get_random_spawn_transform(town_map):
    spawn_transforms = town_map.get_spawn_points()
    return random.choice(spawn_transforms)

def get_random_transform(town_map):
    waypoints = town_map.generate_waypoints(0.1)
    return random.choice(waypoints).transform


def draw_arrow(world, transform, thickness=0.1, arrow_length=1.0, color=(255,0,0), life_time=1.0):
    begin = transform.location + carla.Location(z=2.5)
    theta = np.deg2rad(transform.rotation.yaw)
    end = begin + arrow_length * carla.Location(x=np.cos(theta), y=np.sin(theta))
    world.debug.draw_arrow(begin, end, thickness=thickness, arrow_size=thickness*2, life_time=life_time, color=carla.Color(*color))

def draw_location(world, location, size=0.1, color=(0,255,0), life_time=50):
    world.debug.draw_point(location, size=size, color=carla.Color(*color), life_time=life_time)

def draw_waypoints(world, waypoints, size=0.1, color=(0,255,0), life_time=50):
    for waypoint in waypoints:
        draw_location(world, waypoint.transform.location, size, color, life_time)



def get_waypoint(town_map, transform):
    return town_map.get_waypoint(transform.location)

def get_actor_waypoint(town_map, actor):
    return get_waypoint(town_map, actor.get_transform())


RoadOption = navigation.local_planner.RoadOption
def get_reference_route(town_map, vehicle, distance_range, sampling_resolution):
    distance_range, sampling_resolution = float(distance_range), float(sampling_resolution)
    sampling_number = int(distance_range / sampling_resolution) + 1
    waypoint = get_actor_waypoint(town_map, vehicle)
    return get_reference_route_wrt_waypoint(waypoint, sampling_resolution, sampling_number)

def get_reference_route_wrt_waypoint(waypoint, sampling_resolution, sampling_number):
    next_waypoint = waypoint
    reference_route = [(next_waypoint, RoadOption.LANEFOLLOW)]
    for i in range(1, sampling_number):
        next_waypoint = random.choice(next_waypoint.next(sampling_resolution))
        reference_route.append( (next_waypoint, RoadOption.LANEFOLLOW) )
    return reference_route


file_dir = os.path.dirname(__file__)
def get_spawn_points(town_map, number):
    """
    
    
    Args:
        
    
    Returns:
        list of carla.Location
    """
    
    sp_paths = glob.glob(join(file_dir, './spawn_points/*.txt'))
    file_name = None
    for sp_path in sp_paths:
        map_name = os.path.basename(sp_path).split('.txt')[0]
        if map_name in town_map.name: file_name = sp_path; break
    points = np.loadtxt(file_name)
    length = len(points)
    if length < number:
        print('[get_spawn_points] warning: requested %d vehicles, but could only find %d spawn points' % (number, length))
        number = length
    mask = random.sample(range(length), number)
    sample_points = points[mask,:]
    spawn_points = []
    for sample_point in sample_points:
        spawn_points.append(carla.Location(x=sample_point[0], y=sample_point[1], z=sample_point[2]))
    return spawn_points




def get_route_planner(town_map, sampling_resolution):
    carla_version = get_carla_version()
    if carla_version in new_versions:
        GlobalRoutePlanner = navigation.global_route_planner.GlobalRoutePlanner
        grp = GlobalRoutePlanner(town_map, sampling_resolution)
    else:
        GlobalRoutePlanner = navigation.global_route_planner.GlobalRoutePlanner
        GlobalRoutePlannerDAO = navigation.global_route_planner_dao.GlobalRoutePlannerDAO
        dao = GlobalRoutePlannerDAO(town_map, sampling_resolution)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
    return grp



def get_topology_origin(town_map):
    topology = town_map.get_topology()
    return topology


def get_topology(town_map, sampling_resolution):
    """
    Returns:
        List[Data], contains ['entry', 'exit', 'entryxyz', 'exitxyz', 'path', 'info']
                    'info' contains x, y, z, lane_width
    """
    grp = get_route_planner(town_map, sampling_resolution)
    topology = [Data(**t) for t in grp._topology]

    for t in topology:
        start, end = t.entry.transform.location, t.exit.transform.location
        x, y, z = [start.x], [start.y], [start.z]
        lane_width = [t.entry.lane_width]
        for wp in t.path:
            x.append(wp.transform.location.x)
            y.append(wp.transform.location.y)
            z.append(wp.transform.location.z)
            lane_width.append(wp.lane_width)
        x.append(end.x)
        y.append(end.y)
        z.append(end.z)
        lane_width.append(t.exit.lane_width)
        t.update(info=Data(x=x, y=y, z=z, lane_width=lane_width))
        t.update(waypoints=[t.entry] + t.path + [t.exit])

    return topology


def get_town_map_boundary(town_map, margin=10):
    waypoints = town_map.generate_waypoints(10)
    x_min = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
    y_min = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin
    x_max = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
    y_max = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
    return Data(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)




class HashableJunction(object):
    def __init__(self, junction: carla.Junction):
        self.junction = junction

    def __hash__(self):
        return self.junction.id

    def __eq__(self, other):
        return self.junction.id == other.junction.id



def get_junctions(topology):
    junctions = []
    for t in topology:
        for wp in t.waypoints:
            junction = wp.get_junction()
            if junction != None:
                junctions.append(HashableJunction(junction))
    junctions = [j.junction for j in list(set(junctions))]
    return junctions




def get_weather_name(weather):
    if weather == carla.WeatherParameters.ClearNoon:
        weather_name = 'ClearNoon'
    elif weather == carla.WeatherParameters.ClearSunset:
        weather_name = 'ClearSunset'
    elif weather == carla.WeatherParameters.CloudyNoon:
        weather_name = 'CloudyNoon'
    elif weather == carla.WeatherParameters.CloudySunset:
        weather_name = 'CloudySunset'
    elif weather == carla.WeatherParameters.Default:
        weather_name = 'Default'
    elif weather == carla.WeatherParameters.HardRainNoon:
        weather_name = 'HardRainNoon'
    elif weather == carla.WeatherParameters.HardRainSunset:
        weather_name = 'HardRainSunset'
    elif weather == carla.WeatherParameters.MidRainSunset:
        weather_name = 'MidRainSunset'
    elif weather == carla.WeatherParameters.MidRainyNoon:
        weather_name = 'MidRainyNoon'
    elif weather == carla.WeatherParameters.SoftRainNoon:
        weather_name = 'SoftRainNoon'
    elif weather == carla.WeatherParameters.SoftRainSunset:
        weather_name = 'SoftRainSunset'
    elif weather == carla.WeatherParameters.WetCloudyNoon:
        weather_name = 'WetCloudyNoon'
    elif weather == carla.WeatherParameters.WetCloudySunset:
        weather_name = 'WetCloudySunset'
    elif weather == carla.WeatherParameters.WetNoon:
        weather_name = 'WetNoon'
    elif weather == carla.WeatherParameters.WetSunset:
        weather_name = 'WetSunset'
    else:
        weather_name = str(weather)
    return weather_name


def get_weather_from_name(weather_name):
    if weather_name == 'ClearNoon':
        weather = carla.WeatherParameters.ClearNoon
    elif weather_name == 'ClearSunset':
        weather = carla.WeatherParameters.ClearSunset
    elif weather_name == 'CloudyNoon':
        weather = carla.WeatherParameters.CloudyNoon
    elif weather_name == 'CloudySunset':
        weather = carla.WeatherParameters.CloudySunset
    elif weather_name == 'Default':
        weather = carla.WeatherParameters.Default
    elif weather_name == 'HardRainNoon':
        weather = carla.WeatherParameters.HardRainNoon
    elif weather_name == 'HardRainSunset':
        weather = carla.WeatherParameters.HardRainSunset
    elif weather_name == 'MidRainSunset':
        weather = carla.WeatherParameters.MidRainSunset
    elif weather_name == 'MidRainyNoon':
        weather = carla.WeatherParameters.MidRainyNoon
    elif weather_name == 'SoftRainNoon':
        weather = carla.WeatherParameters.SoftRainNoon
    elif weather_name == 'SoftRainSunset':
        weather = carla.WeatherParameters.SoftRainSunset
    elif weather_name == 'WetCloudyNoon':
        weather = carla.WeatherParameters.WetCloudyNoon
    elif weather_name == 'WetCloudySunset':
        weather = carla.WeatherParameters.WetCloudySunset
    elif weather_name == 'WetNoon':
        weather = carla.WeatherParameters.WetNoon
    elif weather_name == 'WetSunset':
        weather = carla.WeatherParameters.WetSunset
    elif weather_name == 'None':
        weather = carla.WeatherParameters.ClearNoon
    else:
        raise NotImplementedError
    return weather


