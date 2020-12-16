
import carla
SpawnActor = carla.command.SpawnActor

import numpy as np
import random
import time
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


def create_blueprint(world, type_id, **attributes):
    blueprint_lib = world.get_blueprint_library()
    bp = random.choice(blueprint_lib.filter(type_id))

    role_name = attributes.get('role_name', 'hero')
    bp.set_attribute('role_name', role_name)

    if bp.has_attribute('color'):
        color = attributes.get('color', (255,255,255))
        color = str(color[0]) + ',' + str(color[1]) + ',' + str(color[2])
        bp.set_attribute('color', color)
    if bp.has_attribute('driver_id'):
        driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
        bp.set_attribute('driver_id', driver_id)
    if bp.has_attribute('is_invincible'):
        bp.set_attribute('is_invincible', 'true')
    
    return bp

def get_spawn_transform(town_map, spawn_point, height=0.1):
    '''
        only use x,y of spawn_point
    '''
    x, y = spawn_point.x, spawn_point.y
    spawn_transform = town_map.get_waypoint(carla.Location(x=x, y=y)).transform
    spawn_transform.location.z += height
    return spawn_transform


def add_vehicle(world, town_map, spawn_point, type_id='vehicle.bmw.grandtourer', **attributes):
    """
    
    
    Args:
        attributes: contains role_name, color
    
    Returns:
        carla.Vehicle
    """
    
    bp = create_blueprint(world, type_id, **attributes)

    spawn_transforms = town_map.get_spawn_points()
    spawn_transform = random.choice(spawn_transforms) if spawn_transforms else carla.Transform()
    vehicle = world.try_spawn_actor(bp, spawn_transform)
    while vehicle is None:
        spawn_transform = random.choice(spawn_transforms) if spawn_transforms else carla.Transform()
        vehicle = world.try_spawn_actor(bp, spawn_transform)
    if spawn_point:
        spawn_transform = get_spawn_transform(town_map, spawn_point, height=0.2)
        vehicle.set_transform(spawn_transform)
    # world.tick() # TODO check
    tick_world(world)
    print('spawn_point: x={}, y={}'.format(vehicle.get_location().x, vehicle.get_location().y))
    return vehicle


def add_vehicles(client, world, town_map, enable_physics, spawn_points, type_ids, **attributes):
    """
    
    
    Args:
        attributes: contains role_names, colors
    
    Returns:
        list of carla.Vehicle
    """
    number = len(spawn_points)
    role_names = attributes.get('role_names', ['hero']*number)
    colors = attributes.get('colors', [(255,255,255)]*number)
    bps = [create_blueprint(world, type_ids[i], role_name=role_names[i], color=colors[i]) for i in range(number)]
    batch = []
    for bp, spawn_point in zip(bps, spawn_points):
        spawn_transform = get_spawn_transform(town_map, spawn_point, height=0.1)
        batch.append(SpawnActor(bp, spawn_transform))
    
    actor_ids = []
    for response in client.apply_batch_sync(batch):
        if response.error: raise RuntimeError('spawn sensor failed: ' + response.error)
        else: actor_ids.append(response.actor_id)
    vehicles = world.get_actors(actor_ids)
    for vehicle in vehicles: vehicle.set_simulate_physics(enable_physics)
    tick_world(world)
    return vehicles



def get_actor(world, type_id, role_name):
    '''not suitable for multi-agent'''
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id == type_id and actor.attributes['role_name'] == role_name:
            return actor
    return None

def get_attached_actor(actor_list, actor):
    for target_actor in actor_list:
        print(target_actor.id, target_actor.parent)
    print()


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



from ..augment import error_transform

def get_leading_vehicle_unsafe(vehicle, vehicles, reference_waypoints, max_distance):
    """
        Get leading vehicle wrt reference_waypoints or global_path.
        !warning: distances between reference_waypoints cannot exceed any vehicle length.
    
    Args:
        reference_waypoints: list of carla.Waypoint
    
    Returns:
        
    """
    
    current_location = vehicle.get_location()
    vehicle_id = vehicle.id
    vehicle_half_height = vehicle.bounding_box.extent.z
    func = lambda loc: loc.distance(current_location)
    obstacles = [(func(o.get_location()), o) for o in vehicles if o.id != vehicle_id and func(o.get_location()) <= 1.001*max_distance]
    sorted_obstacles = sorted(obstacles, key=lambda x:x[0])

    leading_vehicle, leading_distance = None, 0.0
    for i, waypoint in enumerate(reference_waypoints):
        if i > 0: leading_distance += waypoint.transform.location.distance(reference_waypoints[i-1].transform.location)
        if leading_distance > 1.001*max_distance: break
        location = waypoint.transform.location
        location.z += vehicle_half_height
        for _, obstacle in sorted_obstacles:
            obstacle_transform = obstacle.get_transform()
            if obstacle.bounding_box.contains(location, obstacle_transform):
                leading_vehicle = obstacle
                longitudinal_e, _, _ = error_transform(obstacle_transform, waypoint.transform)
                leading_distance += longitudinal_e
                break
        if leading_vehicle is not None: break
    return leading_vehicle, leading_distance


