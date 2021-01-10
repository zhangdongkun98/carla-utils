import carla

from ..system import parse_yaml_file_unsafe
from ..world_map import connect_to_server

from .tools import generate_args

if __name__ == "__main__":
    import os
    from os.path import join

    try:
        config = parse_yaml_file_unsafe('./config/carla.yaml')
    except FileNotFoundError:
        print('[vehicle_visualizer] use default config.')
        file_dir = os.path.dirname(__file__)
        config = parse_yaml_file_unsafe(join(file_dir, './default_carla.yaml'))
    args = generate_args()
    config.update(args)

    client, world, town_map = connect_to_server(config.host, config.port, config.timeout)

    actors = world.get_actors()
    vehicles = actors.filter('*vehicle*')
    sensors = actors.filter('*sensor*')

    for actor in sensors: print(actor)
    for actor in vehicles: print(actor)

    import pdb; pdb.set_trace()

    for actor in sensors: actor.destroy()
    for actor in vehicles: actor.destroy()

