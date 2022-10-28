import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import importlib

from . import system

'''import carla'''
if not system.load_carla_py():
    system.load_carla()
try:
    carla = importlib.import_module('carla')
except Exception as e:
    print('Fail to import carla')
    raise RuntimeError(e)
print('carla:', carla)
navigation = importlib.import_module('agents.navigation')
importlib.import_module('agents.navigation.local_planner')
importlib.import_module('agents.navigation.global_route_planner')
try:
    importlib.import_module('agents.navigation.global_route_planner_dao')
except:
    pass


'''import modules'''
from .system import parse_json_file, parse_yaml_file, parse_yaml_file_unsafe, printVariable, NCQPipe
from .system import kill_process

from . import basic


'''examples'''
from .examples import NPC

'''augment'''
from .augment import vector3DToArray
from .augment import ActorVertices, CollisionCheck
from .augment import *

from . import trajectory

from .world_map import *
from .world_map import default_settings
from .world_map import add_vehicle, add_vehicles
from .world_map import get_spawn_points
from .world_map import Role
from .world_map import kill_all_servers
from .world_map import get_topology, get_topology_origin
from .world_map import get_junctions

'''sensor'''
from .sensor import sensor_frame_id
from .sensor import createSensorListMaster, createSensorListMasters, CarlaSensorMaster, CarlaSensorListMaster
from .sensor import CameraParams, PesudoSensor

from . import sensor


'''agents'''
from .agents import AgentsRoutePlanner, Controller
from .agents import BaseAgent, BaseAgentObstacle, DefaultAgent
from .agents import NaiveAgent, IdmAgent
from .agents import AgentListMaster

'''perception'''
from . import perception

'''others'''
from termcolor import cprint
