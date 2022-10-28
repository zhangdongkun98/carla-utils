
from ..world_map import vehicle_frame_id


def sensor_frame_id(sensor):
    return vehicle_frame_id(sensor.parent) + '_' + sensor.type_id.replace('.', '_') + '_' + sensor.attributes['role_name']

