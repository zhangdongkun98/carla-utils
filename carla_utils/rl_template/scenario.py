from carla_utils import carla

import numpy as np
import pickle
import random
from typing import List, Union

from ..basic import YamlConfig, prefix, Data, pi2pi
from ..augment import GlobalPath
from ..sensor import createSensorListMaster, CarlaSensorListMaster
from ..world_map import Core
from ..world_map import get_spawn_transform, Role, add_vehicle, add_vehicles, default_settings
from ..world_map import get_reference_route_wrt_waypoint
from ..world_map import get_town_map_boundary
from ..world_map import ScenarioRole
from ..agents import AgentListMaster, BaseAgentObstacle, BaseAgent, NaiveAgent



class ScenarioRandomization(object):
    def __init__(self, core, spawn_points, num_vehicles):
        self.core = core
        self.town_map = core.town_map
        self._spawn_points = spawn_points
        self.num_vehicles = num_vehicles

        self.spawn_points = self.get_spawn_points(num_vehicles)
        self.global_paths = [self.generate_global_path(spawn_point) for spawn_point in self.spawn_points]
        return

    def get_spawn_points(self, num_vehicles, *args):
        if num_vehicles > len(self._spawn_points):
            msg = 'requested {} vehicles, but could only find {} spawn points'.format(num_vehicles, len(self._spawn_points))
            print(prefix(self) + 'warning: {}'.format(msg))
        spawn_points = np.random.choice(self._spawn_points, size=num_vehicles, replace=False)
        return spawn_points

    def generate_global_path(self, spawn_point: carla.Location, *args):
        spawn_transform = get_spawn_transform(self.core, spawn_point, height=0.1)
        waypoint = self.town_map.get_waypoint(spawn_transform.location)
        route = get_reference_route_wrt_waypoint(waypoint, sampling_resolution=0.2, sampling_number=500)
        return GlobalPath(route)


    @staticmethod
    def load_from_disk(core, file_path):
        with open(file_path, 'rb') as f:
            picklable_scenario_randomization = pickle.load(f)
        picklable_global_paths = picklable_scenario_randomization.global_paths
        global_paths = []
        for picklable_gp in picklable_global_paths:
            route = []
            for wp, o in picklable_gp.route:
                carla_wp = wp.get_waypoint(core.town_map)
                if wp.transform.location.distance(carla_wp.transform.location) > 0.1:  ### due to the bug of carla.Map.get_waypoint
                    continue
                if np.abs(pi2pi(np.deg2rad(wp.transform.rotation.yaw - carla_wp.transform.rotation.yaw))) > np.deg2rad(30):
                    continue
                route.append((carla_wp, o))
            global_path = GlobalPath(route)
            global_paths.append(global_path)
        picklable_scenario_randomization.global_paths = global_paths
        return picklable_scenario_randomization





class ScenarioSingleAgent(object):
    scenario_randomization_cls = ScenarioRandomization

    boundary = Data(x_min=-np.inf, y_min=-np.inf, x_max=np.inf, y_max=np.inf)

    time_tolerance = None

    map_name = None              ### must be modified
    weather = None
    max_velocity = None
    obstacle_color = None        ### None means random, otherwise, e.g., (255,255,255)
    type_id = None
    obstacle_type_id = None
    agent_role = ScenarioRole.learnable
    obstacle_role = ScenarioRole.obstacle

    num_vehicles = None
    num_vehicles_min = None
    num_vehicles_max = None
    num_walkers = None

    def __init__(self, config):
        self.set_parameters(config)
        self.config = config
        self.core: Core = config.get('core', None)
        self.town_map = self.core.town_map
        self.spawn_points = np.array(self._generate_spawn_points())

        town_map_boundary = get_town_map_boundary(self.town_map)
        if self.boundary.x_min < town_map_boundary.x_min:
            self.boundary.x_min = town_map_boundary.x_min
        if self.boundary.y_min < town_map_boundary.y_min:
            self.boundary.y_min = town_map_boundary.y_min
        if self.boundary.x_max > town_map_boundary.x_max:
            self.boundary.x_max = town_map_boundary.x_max
        if self.boundary.y_max > town_map_boundary.y_max:
            self.boundary.y_max = town_map_boundary.y_max
        return


    @classmethod
    def set_parameters(cls, config: YamlConfig):
        config.set('time_tolerance', cls.time_tolerance)
        config.set('max_velocity', cls.max_velocity)
        config.set('num_vehicles', cls.num_vehicles)
        config.set('num_vehicles_min', cls.num_vehicles_min)
        config.set('num_vehicles_max', cls.num_vehicles_max)
        return


    def get_map(self):
        return self.map_name
    def get_weather(self):
        return self.weather


    def reset(self):
        return

    def _generate_spawn_points(self):
        return [t.location for t in self.town_map.get_spawn_points()]


    def register_agents(self, step_reset, agents_master: AgentListMaster, sensors_params):
        self.step_reset = step_reset

        self.scenario_randomization: ScenarioRandomization = self.get_scenario_randomization()
        spawn_points = self.scenario_randomization.spawn_points
        global_paths = self.scenario_randomization.global_paths

        type_ids = self.type_ids
        colors = self.colors
        role_names = self.role_names

        '''add ego vehicle'''
        ego_vehicle = add_vehicle(self.core, True, spawn_points[0], type_ids[0], role_name=role_names[0], color=colors[0])

        ### route_planner has a bug: global_path.origin != vehicle.get_transform()
        global_path = global_paths[0]
        ego_vehicle.set_transform(global_path.origin)
        self.core.tick()

        sensors_master = createSensorListMaster(self.config, ego_vehicle, sensors_params)

        '''register agent'''
        Agent = agents_master.get_agent_type()
        agent: BaseAgent = Agent(self.config, ego_vehicle, sensors_master, global_path)
        agent.extend_route()
        agents_master.register(agent, learnable=True)
        
        '''add vehicles'''
        vehicles = add_vehicles(self.core, True, spawn_points[1:], type_ids[1:], role_names=role_names[1:], colors=colors[1:])
        for v, gp in zip(vehicles, global_paths[1:]):
            agent_type = Role.loads(v.attributes['role_name']).atype
            learnable = False
            sm = CarlaSensorListMaster(self.core, v)
            if agent_type == ScenarioRole.agent:
                Agent = agents_master.get_agent_type(learnable)
                agent = Agent(self.config, v, sm, gp)
                agents_master.register(agent, learnable)
            elif agent_type == ScenarioRole.obstacle or agent_type == ScenarioRole.static:
                agents_master.register_obs( BaseAgentObstacle(self.config, v, sm) )
            else:
                raise NotImplementedError
        self.core.tick()
        self.register_agents_callback()
        return


    def get_scenario_randomization(self):
        scenario_randomization_cls = self.config.get('scenario_randomization_cls', self.scenario_randomization_cls)
        scenario_randomization = scenario_randomization_cls(self.core, self.spawn_points, self.num_vehicles)
        return scenario_randomization

    def register_agents_callback(self):
        return



    @property
    def type_ids(self):
        _type_ids = [self.type_id] + [self.obstacle_type_id] * (self.num_vehicles-1)
        return _type_ids
    @property
    def colors(self):
        _colors = [(255,0,0)] + [self.obstacle_color] * (self.num_vehicles-1)
        return _colors
    @property
    def role_names(self):
        _role_names = [Role(vi=0, atype=self.agent_role,)]
        _role_names += [Role(vi=vi, atype=self.obstacle_role,) for vi in range(1, self.num_vehicles)]
        return _role_names


