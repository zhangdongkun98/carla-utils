from carla_utils import carla
DestroyActor = carla.command.DestroyActor

import random
import pickle
from enum import Enum

from ..basic import prefix
from .core import Core


def get_actor(world, type_id, role_name):
    """
        ! warning: not suitable for multi-agent
        ! deprecated
    """
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id == type_id and actor.attributes['role_name'] == role_name:
            return actor
    return None

def get_attached_actor(actor_list, actor):
    for target_actor in actor_list:
        print(target_actor.id, target_actor.parent)
    print()



def destroy_actors(core, actors):
    client = core.client
    client.apply_batch([DestroyActor(x) for x in actors])
    return


def destroy_all_actors(core: Core):
    core.tick()
    actors = core.world.get_actors()
    vehicles = actors.filter('*vehicle*')
    sensors = actors.filter('*sensor*')
    walkers = actors.filter('*walker*')
    for actor in sensors: actor.destroy()
    for actor in vehicles: actor.destroy()
    for actor in walkers: actor.destroy()
    core.tick()
    print(prefix(__name__), 'destroy {} sensors, {} vehicles, {} walkers'.format(len(sensors), len(vehicles), len(walkers)))
    return



from ..basic import Data
class Role(Data):
    def dumps(self):
        return pickle.dumps(self, 0).decode()
    
    @staticmethod
    def loads(role_str):
        if isinstance(role_str, Data):
            role = role_str
        else:
            try:
                role = pickle.loads(bytes(role_str, encoding='utf-8'))
            except:
                role = Role(name=role_str, atype=ScenarioRole.obstacle)
        return role



class ScenarioRole(Enum):
    learnable = -1
    obstacle = 1
    agent = 2
    static = 3


