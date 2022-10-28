from carla_utils import carla
SpawnActor = carla.command.SpawnActor
DestroyActor = carla.command.DestroyActor

import random

from .core import Core


def add_walkers(core: Core, num_walkers, ratio_running=0.0, ratio_crossing=0.0, type_id=None):
    client, world, traffic_manager = core.client, core.world, core.traffic_manager

    if type_id == None:
        type_id = 'walker.pedestrian.*'

    spawn_points = []
    for i in range(num_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    
    number = len(spawn_points)
    bps = []
    for i in range(number):
        if (random.random() > ratio_running):
            running = False
        else:
            running = True
        bp = create_blueprint(core, type_id, running=running)
        bps.append(bp)

    batch = [SpawnActor(bp, spawn_point) for (spawn_point, bp) in zip(spawn_points, bps)]

    walker_ids = []
    for response in client.apply_batch_sync(batch):
        if response.error: pass
        else: walker_ids.append(response.actor_id)
    walkers = world.get_actors(walker_ids)


    controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    batch = [SpawnActor(controller_bp, carla.Transform(), walker) for walker in walkers]

    walker_controller_ids = []
    for response in client.apply_batch_sync(batch):
        if response.error: pass
        else: walker_controller_ids.append(response.actor_id)
    walker_controllers = world.get_actors(walker_controller_ids)

    core.tick()

    world.set_pedestrians_cross_factor(ratio_crossing)
    walkers = [WalkerWithController(core, w, c) for w, c in zip(walkers, walker_controllers)]
    [walker.setup() for walker in walkers]

    core.tick()
    return walkers



def create_blueprint(core, type_id, **attributes):
    world = core.world
    blueprint_lib = world.get_blueprint_library()

    blueprint_lib = blueprint_lib.filter(type_id)
    bp = random.choice(blueprint_lib)

    running = attributes.get('running', False)

    if bp.has_attribute('is_invincible'):
        bp.set_attribute('is_invincible', 'false')
    if bp.has_attribute('speed'):
        if not running:
            speed = bp.get_attribute('speed').recommended_values[1]
        else:
            speed = bp.get_attribute('speed').recommended_values[2]
        bp.set_attribute('speed', speed)
    return bp
    



class WalkerWithController(object):
    def __init__(self, core: Core, walker: carla.Walker, controller: carla.WalkerAIController):
        self.core = core
        self.walker = walker
        self.controller = controller

    def setup(self):
        speed = eval(self.walker.attributes.get('speed', 1.4))
        self.controller.start()
        self.controller.go_to_location(self.core.world.get_random_location_from_navigation())
        self.controller.set_max_speed(speed)

    def destroy_commands(self):
        self.controller.stop()
        return [DestroyActor(self.controller), DestroyActor(self.walker)]


