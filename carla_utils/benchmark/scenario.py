import carla_utils as cu
from carla_utils import carla
from carla_utils.rl_template import ScenarioRandomization, ScenarioSingleAgent


import os

import numpy as np
import random




class ScenarioRandomizationNoCrashBenchmark(ScenarioRandomization):
    """
        Only generate single global path for ego agent.
    """

    def __init__(self, core, spawn_points, num_vehicles, weathers):
        self.core = core
        self.town_map = core.town_map
        self.map_name = core.map_name
        self._spawn_points = spawn_points
        self.num_vehicles = num_vehicles

        config = cu.basic.YamlConfig(core=core, sampling_resolution=1.0)
        self.route_planner = cu.AgentsRoutePlanner(config)

        dir_path = os.path.split(os.path.abspath(__file__))[0]
        with open(os.path.join(dir_path, f'suite/nocrash_{self.map_name}.txt'), 'r') as f:
            spawn_point_indices = [tuple(map(int, l.split())) for l in f.readlines()]

        index = np.random.randint(0, len(spawn_point_indices))
        start_index, end_index = spawn_point_indices[index]
        start, end = spawn_points[start_index], spawn_points[end_index]
        self.global_paths = [self.generate_global_path(start, end)] + [None] *(num_vehicles-1)

        self.spawn_points = self.get_spawn_points(num_vehicles, start)
        self.weather = random.choice(weathers)
        return

    def get_spawn_points(self, num_vehicles, spawn_point):
        spawn_points = np.random.choice(self._spawn_points, size=num_vehicles-1, replace=False)
        return [spawn_point] + [sp for sp in spawn_points]

    def generate_global_path(self, start: carla.Location, end: carla.Location):
        origin = self.town_map.get_waypoint(start).transform.location
        destination = self.town_map.get_waypoint(end).transform.location
        return self.route_planner.trace_route(origin, destination)





class ScenarioNoCrashBenchmark(ScenarioSingleAgent):
    scenario_randomization_cls = ScenarioRandomizationNoCrashBenchmark

    @staticmethod
    def get_config_scenario(mode_town, mode_weather, mode_traffic):
        map_name = 'Town01' if mode_town == 'train' else 'Town02'
        weathers = [
            'ClearNoon',
            'WetNoon',
            'HardRainNoon',
            'ClearSunset',
        ] if mode_weather == 'train' else [
            'WetSunset',
            'SoftRainSunset',
        ]
        weather = weathers[0]

        if mode_traffic == 'empty':
            if map_name == 'Town01':
                num_vehicles = 0 +1
                num_walkers = 0
            else:
                num_vehicles = 0 +1
                num_walkers = 0
        elif mode_traffic == 'regular':
            if map_name == 'Town01':
                num_vehicles = 20 +1
                num_walkers = 50
            else:
                num_vehicles = 15 +1
                num_walkers = 50
        elif mode_traffic == 'dense':
            if map_name == 'Town01':
                num_vehicles = 100 +1
                num_walkers = 200
            else:
                num_vehicles = 70 +1
                num_walkers = 150
        else:
            raise NotImplementedError
        num_vehicles_min = num_vehicles_max = num_vehicles
        return cu.basic.Data(
            map_name=map_name,
            weathers=weathers, weather=weather,
            num_vehicles=num_vehicles, num_vehicles_min=num_vehicles_min, num_vehicles_max=num_vehicles_max,
            num_walkers=num_walkers,
        )
    

    mode_town = ['train', 'evaluate'][0]
    mode_weather = ['train', 'evaluate'][0]
    mode_traffic = ['empty', 'regular', 'dense'][-1]

    config_scenario = get_config_scenario.__func__(mode_town, mode_weather, mode_traffic)

    map_name = config_scenario.map_name
    weathers = config_scenario.weathers
    weather = config_scenario.weather
    num_vehicles = config_scenario.num_vehicles
    num_vehicles_min = config_scenario.num_vehicles_min
    num_vehicles_max = config_scenario.num_vehicles_max
    num_walkers = config_scenario.num_walkers


    time_tolerance = 1000  ### no use
    max_velocity = 8.0
    type_id = 'vehicle.tesla.model3'
    obstacle_type_id = 'vehicle.*'



    def get_scenario_randomization(self):
        scenario_randomization_cls = self.config.get('scenario_randomization_cls', self.scenario_randomization_cls)
        scenario_randomization = scenario_randomization_cls(self.core, self.spawn_points, self.num_vehicles, self.weathers)
        return scenario_randomization


    def register_agents(self, step_reset, agents_master, sensors_params):
        super().register_agents(step_reset, agents_master, sensors_params)

        walkers = cu.add_walkers(self.core, self.num_walkers, ratio_crossing=1.0)
        [agents_master.register_walker(walker) for walker in walkers]

        self.weather = self.scenario_randomization.weather  ### next weather

        self.core.tick()
        return





