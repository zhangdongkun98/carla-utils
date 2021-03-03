
import random
import pickle


def get_actor(world, type_id, role_name):
    '''not suitable for multi-agent'''   ### !warning TODO
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id == type_id and actor.attributes['role_name'] == role_name:
            return actor
    return None

def get_attached_actor(actor_list, actor):
    for target_actor in actor_list:
        print(target_actor.id, target_actor.parent)
    print()


def create_blueprint(world, type_id, **attributes):
    blueprint_lib = world.get_blueprint_library()
    bp = random.choice(blueprint_lib.filter(type_id))

    role_name: Role = attributes.get('role_name', Role(name='hero'))
    bp.set_attribute('role_name', role_name.dumps())

    if bp.has_attribute('color'):
        color = attributes.get('color', (255,255,255))
        color = str(color[0]) + ',' + str(color[1]) + ',' + str(color[2])
        bp.set_attribute('color', color)
    if bp.has_attribute('driver_id'):
        driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
        bp.set_attribute('driver_id', driver_id)
    if bp.has_attribute('is_invincible'):
        bp.set_attribute('is_invincible', 'true')
    return bp


class Role(object):
    def __init__(self, **kwargs):
        for (key, value) in kwargs.items():
            setattr(self, key, value)
        return

    def dumps(self):
        return pickle.dumps(self, 0).decode()
    
    @staticmethod
    def loads(role_str):
        return pickle.loads(bytes(role_str, encoding='utf-8'))
    
    def __str__(self):
        block_words = ['dumps', 'loads']

        res = 'Role('
        for attribute in dir(self):
            if attribute in block_words: continue
            if not attribute.startswith('_'):
                value = getattr(self, attribute)
                res += str(attribute) + '=' + str(value) + ', '
        return res[:-2] + ')'


