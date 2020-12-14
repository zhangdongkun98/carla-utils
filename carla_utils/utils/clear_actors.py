import carla

from ..system import parse_yaml_file_unsafe
from ..world_map import connect_to_server

if __name__ == "__main__":
    config = parse_yaml_file_unsafe('./config/carla.yaml')
    client, world, town_map = connect_to_server(config.host, config.port, config.timeout)

    actors = world.get_actors()
    vehicles = actors.filter('*vehicle*')
    sensors = actors.filter('*sensor*')

    for actor in sensors: print(actor)
    for actor in vehicles: print(actor)

    import pdb; pdb.set_trace()

    for actor in sensors: actor.destroy()
    for actor in vehicles: actor.destroy()

