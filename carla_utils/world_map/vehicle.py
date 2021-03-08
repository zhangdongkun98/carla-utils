import carla
SpawnActor = carla.command.SpawnActor


import random

from .tools import get_spawn_transform, tick_world
from .actor import create_blueprint, Role


def vehicle_frame_id(vehicle):
    vfid = vehicle.type_id.replace('.', '_') + '_'
    try:
        role = Role.loads(vehicle.attributes['role_name'])
        vfid += role.name + '_' + str(role.vi)
    except:
        vfid += vehicle.attributes['role_name'] + '_' + str(vehicle.id)
    return vfid


def add_vehicle(world, town_map, enable_physics, spawn_point, type_id='vehicle.bmw.grandtourer', **attributes):
    """
    
    
    Args:
        attributes: contains role_name, color
    
    Returns:
        carla.Vehicle
    """
    
    bp = create_blueprint(world, type_id, **attributes)

    spawn_transforms = town_map.get_spawn_points()
    spawn_transform = random.choice(spawn_transforms) if spawn_transforms else carla.Transform()
    if spawn_point:
        spawn_transform = get_spawn_transform(town_map, spawn_point, height=0.2)
    vehicle = world.spawn_actor(bp, spawn_transform)
    vehicle.set_simulate_physics(enable_physics)
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


