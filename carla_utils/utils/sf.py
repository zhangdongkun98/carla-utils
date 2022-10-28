'''
    server_fps
'''

from carla_utils import carla

import copy
import pygame

from ..system import parse_yaml_file_unsafe, Clock
from ..world_map import Core

from .tools import default_argparser
import sys


class WorldTick(object):
    def __init__(self, core: Core):
        self.core  = core
        self.world = core.world

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


class ServerFps(object):
    def __init__(self, config):
        config.set('mode', 'None')

        self.world_ticks = {}
        if config.batch:
            ports = config.ports
        else:
            ports = [config.port]
        for port in ports:
            cfg = copy.copy(config)
            cfg.port = port
            core = Core(cfg, use_tm=False)
            world_tick = WorldTick(core)
            self.world_ticks[port] = world_tick

        self.clock = Clock(2)

    
    def run_step(self):
        res = ''
        for port, world_tick in self.world_ticks.items():
            res += 'port {}: {:0=6.2f}, '.format(port, world_tick.server_fps)
        res = res[:-2]
        print(res, end='\r', flush=True)
        return

    def run(self):
        while True:
            self.clock.tick_begin()
            self.run_step()
            self.clock.tick_end()


if __name__ == "__main__":
    print(__doc__)

    import os
    from os.path import join

    try:
        config = parse_yaml_file_unsafe('./config/carla.yaml')
    except FileNotFoundError:
        print('[vehicle_visualizer] use default config.')
        file_dir = os.path.dirname(__file__)
        config = parse_yaml_file_unsafe(join(file_dir, './default_carla.yaml'))
    argparser = default_argparser()
    argparser.add_argument('--batch', action='store_true', help='batch viz env (default: False)')
    argparser.add_argument('--ports', default=[2000,2002,2004,2006], type=int, nargs='+', help='')
    args = argparser.parse_args()
    config.update(args)
    
    server_fps = ServerFps(config)
    try:
        server_fps.run()
    except KeyboardInterrupt:
        print('\ncanceled by user')

