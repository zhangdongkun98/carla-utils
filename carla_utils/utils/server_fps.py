import carla

import pygame

from ..system import parse_yaml_file_unsafe, Clock
from ..world_map import connect_to_server

class ServerFps(object):
    def __init__(self, config):
        client, self.world, town_map = connect_to_server(config.host, config.port, config.timeout)
        self.clock = Clock(2)

        self.world.on_tick(self.on_world_tick)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._server_clock = pygame.time.Clock()


    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds
    
    def run_step(self):
        print(self.server_fps)

    def run(self):
        while True:
            t1 = self.clock.tick_begin()

            self.run_step()

            t2 = self.clock.tick_end()


if __name__ == "__main__":
    config = parse_yaml_file_unsafe('./config/carla.yaml')
    server_fps = ServerFps(config)
    try:
        server_fps.run()
    except KeyboardInterrupt:
        print('canceled by user')
