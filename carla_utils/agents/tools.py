import carla

import numpy as np

from .. import basic
from ..augment import error_transform, vectorYawRad


def get_leading_agent_unsafe(agent, agents, reference_waypoints, max_distance):
    """
        Get leading vehicle wrt reference_waypoints or global_path.
        !warning: distances between reference_waypoints cannot exceed any vehicle length.
    
    Args:
        reference_waypoints: list of carla.Waypoint
    
    Returns:
        
    """
    
    current_location = agent.get_transform().location
    vehicle_id = agent.id
    vehicle_half_width, vehicle_half_height = agent.vehicle.bounding_box.extent.y, agent.vehicle.bounding_box.extent.z
    func = lambda t: t.location.distance(current_location)
    obstacles = [(func(o.get_transform()), o) for o in agents if o.id != vehicle_id and func(o.get_transform()) <= 1.001*max_distance]
    sorted_obstacles = sorted(obstacles, key=lambda x:x[0])

    leading_agent, leading_distance = None, 0.0
    for i, waypoint in enumerate(reference_waypoints):
        if i > 0: leading_distance += waypoint.transform.location.distance(reference_waypoints[i-1].transform.location)
        if leading_distance > 1.001*max_distance: break
        location_c = waypoint.transform.location
        location_l , location_r = side_location_2d(waypoint.transform, vehicle_half_width)
        location_c.z += vehicle_half_height
        location_l.z += vehicle_half_height
        location_r.z += vehicle_half_height
        for _, obstacle in sorted_obstacles:
            obstacle_transform = obstacle.get_transform()
            if      obstacle.vehicle.bounding_box.contains(location_c, obstacle_transform) or \
                    obstacle.vehicle.bounding_box.contains(location_l, obstacle_transform) or \
                    obstacle.vehicle.bounding_box.contains(location_r, obstacle_transform):
                leading_agent = obstacle
                longitudinal_e, _, _ = error_transform(obstacle_transform, waypoint.transform)
                leading_distance += longitudinal_e
                break
        if leading_agent is not None: break
    return leading_agent, leading_distance


def calculate_direction(start_transform, end_transform):
    theta = basic.pi2pi(vectorYawRad(end_transform.location - start_transform.location) - np.deg2rad(start_transform.rotation.yaw))
    threshold = np.deg2rad(10)
    if abs(theta) < threshold: direction = 1
    elif theta > threshold: direction = 0
    else: direction = 2
    return direction


def side_location_2d(transform, half_width):
    center = np.array([transform.location.x, transform.location.y])
    theta = np.deg2rad(transform.rotation.yaw)
    direction = np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    left, right = center + half_width * direction, center - half_width * direction
    return carla.Location(left[0], left[1]), carla.Location(right[0], right[1])
