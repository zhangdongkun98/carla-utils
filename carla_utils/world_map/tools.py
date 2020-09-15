
import carla

import random
import time

def connect_to_server(host, port, timeout=2.0, map_name=None):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    available_map_names = client.get_available_maps()
    world = client.get_world()
    town_map = world.get_map()

    flag1 = town_map.name not in map_name
    flag2 = True in [map_name in available_map_name for available_map_name in available_map_names]
    if flag1 and flag2:
        client.load_world(map_name)
        world = client.get_world()
        town_map = world.get_map()
    print('connected to server {}:{}'.format(host, port))
    return client, world, town_map

def set_weather(world, weather):
    world.set_weather(weather)
    return weather


def add_vehicle(world, role_name, type_id='vehicle.bmw.grandtourer', spawn_point=None):
    town_map = world.get_map()
    blueprint_lib = world.get_blueprint_library()
    bp = random.choice(blueprint_lib.filter(type_id))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    bp.set_attribute('role_name', role_name)

    spawn_transforms = town_map.get_spawn_points()
    spawn_transform = random.choice(spawn_transforms) if spawn_transforms else carla.Transform()
    vehicle = world.try_spawn_actor(bp, spawn_transform)
    while vehicle is None:
        spawn_transform = random.choice(spawn_transforms) if spawn_transforms else carla.Transform()
        vehicle = world.try_spawn_actor(bp, spawn_transform)
    if spawn_point:
        respawn_vehicle(town_map, vehicle, spawn_point)
    time.sleep(0.1)
    print('spawn_point: x={}, y={}'.format(vehicle.get_location().x, vehicle.get_location().y))
    return vehicle

def respawn_vehicle(town_map, vehicle, spawn_point):
    '''
        only use x,y of spawn_point
    '''
    x, y = spawn_point.x, spawn_point.y
    spawn_transform = town_map.get_waypoint(carla.Location(x=x, y=y)).transform
    spawn_transform.location.z += 3
    vehicle.set_transform(spawn_transform)


def get_actor(world, type_id, role_name):
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id == type_id and actor.attributes['role_name'] == role_name:
            return actor
    return None


def remove_traffic_light(vehicle):
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            traffic_light.set_state(carla.TrafficLightState.Green)



def get_random_spawn_transform(town_map):
    spawn_transforms = town_map.get_spawn_points()
    return random.choice(spawn_transforms)


def draw_location(world, location, size=0.1, color=carla.Color(0,255,0,255), life_time=50):
    world.debug.draw_point(location, size=size, color=color, life_time=life_time)

def draw_waypoints(world, waypoints, size=0.1, color=carla.Color(0,255,0,255), life_time=50):
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
    next_waypoint = waypoint
    reference_route = [(next_waypoint, RoadOption.LANEFOLLOW)]
    for i in range(1, sampling_number):
        next_waypoint = next_waypoint.next(sampling_resolution)[0]
        reference_route.append( (next_waypoint, RoadOption.LANEFOLLOW) )
    return reference_route


def get_reference_route_wrt_waypoint(waypoint, sampling_resolution, sampling_number):
    next_waypoint = waypoint
    reference_route = []
    for i in range(1, sampling_number+1):
        next_waypoint = next_waypoint.next(sampling_resolution)[0]
        reference_route.append( (next_waypoint, RoadOption.LANEFOLLOW) )
    return reference_route