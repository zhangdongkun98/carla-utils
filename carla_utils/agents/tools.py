
from ..augment import error_transform


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
    vehicle_half_height = agent.vehicle.bounding_box.extent.z
    func = lambda t: t.location.distance(current_location)
    obstacles = [(func(o.get_transform()), o) for o in agents if o.id != vehicle_id and func(o.get_transform()) <= 1.001*max_distance]
    sorted_obstacles = sorted(obstacles, key=lambda x:x[0])

    leading_agent, leading_distance = None, 0.0
    for i, waypoint in enumerate(reference_waypoints):
        if i > 0: leading_distance += waypoint.transform.location.distance(reference_waypoints[i-1].transform.location)
        if leading_distance > 1.001*max_distance: break
        location = waypoint.transform.location
        location.z += vehicle_half_height
        for _, obstacle in sorted_obstacles:
            obstacle_transform = obstacle.get_transform()
            if obstacle.vehicle.bounding_box.contains(location, obstacle_transform):
                leading_agent = obstacle
                longitudinal_e, _, _ = error_transform(obstacle_transform, waypoint.transform)
                leading_distance += longitudinal_e
                break
        if leading_agent is not None: break
    return leading_agent, leading_distance