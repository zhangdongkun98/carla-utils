import carla

import random
import pygame
import os
from os.path import join

from ..system import parse_yaml_file_unsafe
from ..sensor import createSensorListMaster
from ..world_map import connect_to_server, add_vehicle, Role, Scenario

from .sensor import sensors_params
from .pygame_interaction import PyGameInteraction
from .tools import generate_args



class EgoVehicle(object):
    def __init__(self, config, scenario_cls=Scenario, sensors_params=sensors_params):
        '''parameter'''

        self.frequency = 20
        self.clock = pygame.time.Clock()
        
        host, port, timeout, map_name = config.host, config.port, config.timeout, config.map_name

        client, world, town_map = connect_to_server(host, port, timeout, map_name)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        self.world, self.town_map = world, town_map
        self.scenario = scenario_cls(town_map)

        self.global_frame_id, self.vehicle_frame_id = 'odom', 'vehicle'
        role_name, type_id = config.role_name, config.type_id
        spawn_point = random.choice(self.scenario.spawn_points)
        self.vehicle = add_vehicle(world, town_map, spawn_point, type_id, role_name=Role(vi=0, name=role_name))
        self.sensor_manager = createSensorListMaster(client, world, self.vehicle, sensors_params)
        print('[ego_vehicle] ego_vehicle id: ', self.vehicle.id)

        self.pygame_interaction = PyGameInteraction(config, client, self.vehicle, self.sensor_manager)
        return
        

    def destroy(self):
        self.pygame_interaction.destroy()

        if self.sensor_manager is not None:
            self.sensor_manager.destroy()
            self.sensor_manager = None
        if self.vehicle is not None:
            self.vehicle.destroy()
            self.vehicle = None


    def run_step(self):
        pass

    def run(self):
        while True:
            self.clock.tick_busy_loop(self.frequency)
            self.run_step()
            self.pygame_interaction.tick()
 


if __name__ == '__main__':
    try:
        config = parse_yaml_file_unsafe('./config/carla.yaml')
    except FileNotFoundError:
        print('[vehicle_visualizer] use default config.')
        file_dir = os.path.dirname(__file__)
        config = parse_yaml_file_unsafe(join(file_dir, './default_carla.yaml'))
    args = generate_args()
    config.update(args)

    ego_vehicle = EgoVehicle(config)
    try:
        ego_vehicle.run()
    except KeyboardInterrupt:
        print('canceled by user')
    finally:
        ego_vehicle.destroy()
        print('destroyed all relevant actors')

