from carla_utils import carla

import os
from os.path import join
import random
import signal
import subprocess
import time
import psutil, pynvml

from carla_utils.basic import YamlConfig, Data, prefix
from carla_utils.system import is_used, get_carla_version, new_versions

from .tools import get_weather_name, get_weather_from_name



class Core(object):
    '''
        Inspired by https://github.com/carla-simulator/rllib-integration/blob/main/rllib_integration/carla_core.py
    '''
    def __init__(self, config: YamlConfig, map_name=None, settings=None, use_tm=False):
        if config.get('mode', None) == 'evaluate':
            env_index = config.env_index
            self.server = launch_server(env_index, gpu_index=0, start_port=config.port, sleep_time=20, low_quality=False)
            config.port = self.server.port
        
        self.host, self.port = config.host, int(config.port)
        self.timeout = config.get('timeout', 2.0)
        self.seed = config.get('seed', 0)
        self.mode = config.get('mode', None)

        self.connect_to_server()

        self.available_map_names = self.client.get_available_maps()
        if settings != None:
            self.settings = settings

        if use_tm:
            self.add_trafficmanager()
            self.load_map(map_name)

        config.set('core', self)


    def connect_to_server(self):
        """Connect to the client"""

        num_iter = 10
        for i in range(num_iter):
            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(self.timeout)
                self.world = self.client.get_world()
                self.town_map = self.world.get_map()
                self.map_name = self.town_map.name.split('/')[-1]
                self.settings = self.world.get_settings()
                self.weather = get_weather_name(self.world.get_weather())
                print(prefix(self) + 'connected to server {}:{}'.format(self.host, self.port))
                return
            except Exception as e:
                print(prefix(self) + 'Waiting for server to be ready: {}, attempt {} of {}'.format(e, i + 1, num_iter))
                time.sleep(2)
        raise Exception("Cannot connect to server. Try increasing 'timeout' or 'retries_on_error' at the carla configuration")


    def load_map(self, map_name=None, weather=None):
        ### map
        map_name = str(map_name)
        flag1 = self.map_name not in map_name
        flag2 = True in [map_name in available_map_name for available_map_name in self.available_map_names]
        if flag1 and flag2:
            self.client.load_world(map_name)
            self.world = self.client.get_world()
            self.town_map = self.world.get_map()
            self.map_name = self.town_map.name.split('/')[-1]
            self.weather = get_weather_name(self.world.get_weather())
            print(prefix(self) + 'load map: ', self.map_name)

        ### weather
        weather = str(weather)
        if weather != self.weather:
            self.world.set_weather(get_weather_from_name(weather))
            self.weather = weather
            print(prefix(self) + 'set weather: ', get_weather_name(get_weather_from_name(weather)))

        ### settings
        current_settings = self.world.get_settings()
        if  self.settings.synchronous_mode != current_settings.synchronous_mode \
                or self.settings.no_rendering_mode != current_settings.no_rendering_mode \
                or self.settings.fixed_delta_seconds != current_settings.fixed_delta_seconds:
            self.world.apply_settings(self.settings)
            print(prefix(self) + 'set settings: ', self.settings)

        return


    def add_trafficmanager(self):
        tm_port = self.port + 6000

        # def is_used_by_carla(tm_port):
        #     server_port_pids = [conn.pid for conn in psutil.net_connections() if conn.laddr.port == self.port]
        #     tm_port_pids = [conn.pid for conn in psutil.net_connections() if conn.laddr.port == tm_port]
        if is_used(tm_port):
            msg =   'The desired tm_port is used. There are two possible reasons. ' + \
                    'First, Another main client is connected to the server and occupies this tm_port. ' + \
                    'Next, another process is occupying this tm_port. ' + \
                    'To fix this, launch another server.'
            raise RuntimeError(msg)
        traffic_manager = self.client.get_trafficmanager(tm_port)
        
        if hasattr(traffic_manager, 'set_random_device_seed'):
            traffic_manager.set_random_device_seed(self.seed)
        traffic_manager.set_synchronous_mode(self.settings.synchronous_mode)
        # traffic_manager.set_hybrid_physics_mode(True)  ## do not use this

        self.traffic_manager = traffic_manager
        self.tm_port = tm_port
        return
    

    def tick(self):
        if self.settings.synchronous_mode:
            return self.world.tick()


    def kill(self):
        if hasattr(self, 'server'):
            kill_server(self.server)
        return
    


# =============================================================================
# -- server  ------------------------------------------------------------------
# =============================================================================


def launch_server(env_index, gpu_index=0, host='127.0.0.1', start_port=2000, sleep_time=5.0, low_quality=True, no_display=True):
    port = start_port + env_index *2

    time.sleep(random.uniform(0, 1))

    port = get_port(port)

    cmd = generate_server_cmd(port, gpu_index, low_quality=low_quality, no_display=no_display)
    print(prefix(__name__) + 'running: ', cmd)
    server_process = subprocess.Popen(cmd,
        shell=True,
        preexec_fn=os.setsid,
        stdout=open(os.devnull, 'w'),
    )

    time.sleep(sleep_time)
    server = Data(host=host, port=port, process=server_process)
    return server


def launch_servers(env_indices, gpu_indices=None, sleep_time=20.0):
    if gpu_indices == None:
        gpu_indices = [0] *len(env_indices)
    host = '127.0.0.1'
    servers = []
    for env_index, gpu_index in zip(env_indices, gpu_indices):
        server = launch_server(env_index, gpu_index, host, sleep_time=0.0)
        servers.append(server)
    time.sleep(sleep_time)
    return servers


def kill_server(server):
    server.process.send_signal(signal.SIGKILL)
    os.killpg(os.getpgid(server.process.pid), signal.SIGKILL)
    print(prefix(__name__) + 'killed server {}:{}'.format(server.host, server.port))
    return

def kill_servers(servers):
    for server in servers:
        kill_server(server)
    return

def kill_all_servers():
    '''Kill all PIDs that start with Carla'''
    processes = [p for p in psutil.process_iter() if "carla" in p.name().lower()]
    for process in processes:
        os.kill(process.pid, signal.SIGKILL)


def generate_server_cmd(port, gpu_index=0, low_quality=True, use_opengl=True, no_display=True):
    assert port % 2 == 0

    carla_version = get_carla_version()
    if carla_version in new_versions:
        use_opengl = False

    cmd = join(os.environ['CARLAPATH'], 'CarlaUE4.sh')

    cmd += ' -carla-rpc-port=' + str(port)
    cmd = 'SDL_HINT_CUDA_DEVICE={} '.format(gpu_index) + cmd
    if low_quality:
        cmd += ' -quality-level=Low'
    if use_opengl:
        cmd += ' -opengl'
    if no_display:
        if carla_version in new_versions:
            cmd = cmd + ' -RenderOffScreen'
        else:
            # cmd = 'DISPLAY= ' + cmd   ### deprecated
            cmd = 'SDL_VIDEODRIVER=offscreen ' + cmd
    return cmd



def connect_to_server(host, port, timeout=2.0, map_name=None, **kwargs):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    available_map_names = client.get_available_maps()
    world = client.get_world()
    town_map = world.get_map()

    ### map
    map_name = str(map_name)
    flag1 = town_map.name not in map_name
    flag2 = True in [map_name in available_map_name for available_map_name in available_map_names]
    if flag1 and flag2:
        client.load_world(map_name)
        world = client.get_world()
        town_map = world.get_map()

    ### weather
    weather = kwargs.get('weather', carla.WeatherParameters.ClearNoon)
    world.set_weather(weather)

    ### settings
    current_settings = world.get_settings()

    settings = kwargs.get('settings', current_settings)

    if  settings.synchronous_mode != current_settings.synchronous_mode \
            or settings.no_rendering_mode != current_settings.no_rendering_mode \
            or settings.fixed_delta_seconds != current_settings.fixed_delta_seconds:
        world.apply_settings(settings)
    settings = world.get_settings()

    print('connected to server {}:{}'.format(host, port))
    return client, world, town_map


def get_port(port):
    while is_used(port) or is_used(port+1):
        port += 1000
    return port



# =============================================================================
# -- setting  -----------------------------------------------------------------
# =============================================================================



def default_settings(sync=False, render=True, dt=0.0):
    settings = carla.WorldSettings()
    settings.synchronous_mode = sync
    settings.no_rendering_mode = not render
    settings.fixed_delta_seconds = dt
    return settings


