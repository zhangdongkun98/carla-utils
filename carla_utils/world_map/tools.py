
import carla

import numpy as np
import random
import os, glob
from os.path import join


def tick_world(world):
    if world.get_settings().synchronous_mode: world.tick()

def connect_to_server(host, port, timeout=2.0, map_name=None, **kwargs):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    available_map_names = client.get_available_maps()
    world = client.get_world()
    town_map = world.get_map()

    map_name = str(map_name)
    flag1 = town_map.name not in map_name
    flag2 = True in [map_name in available_map_name for available_map_name in available_map_names]
    if flag1 and flag2:
        client.load_world(map_name)
        world = client.get_world()
        town_map = world.get_map()

    weather = kwargs.get('weather', carla.WeatherParameters.ClearNoon)
    world.set_weather(weather)

    current_settings = world.get_settings()
    settings = kwargs.get('settings', current_settings)
    if  settings.synchronous_mode != current_settings.synchronous_mode \
            or settings.no_rendering_mode != current_settings.no_rendering_mode \
            or settings.fixed_delta_seconds != current_settings.fixed_delta_seconds:
        world.apply_settings(settings)

    print('connected to server {}:{}'.format(host, port))
    return client, world, town_map

def set_weather(world, weather):
    world.set_weather(weather)
    return weather


def get_spawn_transform(town_map, spawn_point, height=0.1):
    '''
        only use x,y of spawn_point
    '''
    x, y = spawn_point.x, spawn_point.y
    spawn_transform = town_map.get_waypoint(carla.Location(x=x, y=y)).transform
    spawn_transform.location.z += height
    return spawn_transform



def remove_traffic_light(vehicle):
    '''
        @todo
    '''
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            traffic_light.set_state(carla.TrafficLightState.Green)



def get_random_spawn_transform(town_map):
    spawn_transforms = town_map.get_spawn_points()
    return random.choice(spawn_transforms)


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



def get_waypoint(town_map, vehicle):
    location = vehicle.get_location()
    return town_map.get_waypoint(location)



from agents.navigation.local_planner import RoadOption
def get_reference_route(town_map, vehicle, distance_range, sampling_resolution):
    distance_range, sampling_resolution = float(distance_range), float(sampling_resolution)
    sampling_number = int(distance_range / sampling_resolution) + 1
    waypoint = get_waypoint(town_map, vehicle)
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


