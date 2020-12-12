import carla

import numpy as np
import weakref
import copy
import time
import logging

from ..system import debug
from ..basic import flatten_list
from .sensor_callback import CarlaSensorCallback
from .sensor_create import create_sensor, create_sensor_command


def createSensorListMaster(client, world, vehicle, sensors_param_list):
    blueprint_library = world.get_blueprint_library()
    batch = [create_sensor_command(world, vehicle, blueprint_library, config) for config in sensors_param_list]
    sensor_ids = []
    for response in client.apply_batch_sync(batch):
        if response.error: raise RuntimeError('spawn sensor failed: ' + response.error)
        else: sensor_ids.append(response.actor_id)
    sensors = world.get_actors(sensor_ids)

    sensors_master = CarlaSensorListMaster(world, vehicle)
    for sensor, config in zip(sensors, sensors_param_list):
        transform, callback = config['transform'], config['callback']
        sensors_master.append(sensor, transform, callback)
    return sensors_master


def createSensorListMasters(client, world, vehicles, sensors_param_lists):
    blueprint_library = world.get_blueprint_library()
    sensors_master_dict = {vehicle.id: CarlaSensorListMaster(world, vehicle) for vehicle in vehicles}

    batch = []
    for vehicle, sensors_param_list in zip(vehicles, sensors_param_lists):
        batch.extend([create_sensor_command(world, vehicle, blueprint_library, config) for config in sensors_param_list])
    
    sensor_ids = []
    for response in client.apply_batch_sync(batch):
        if response.error: raise RuntimeError('spawn sensor failed: ' + response.error)
        else: sensor_ids.append(response.actor_id)
    sensors = world.get_actors(sensor_ids)

    sensors_param_list = flatten_list(sensors_param_lists)
    for sensor, config in zip(sensors, sensors_param_list):
        transform, callback = config['transform'], config['callback']
        sensors_master_dict[sensor.parent.id].append(sensor, transform, callback)
    return list(sensors_master_dict.values())



class CarlaSensorListMaster(object):
    def __init__(self, world, vehicle):
        self.world, self.vehicle = world, vehicle

        self.sensor_list = []
        self.sensor_dict = dict()

        '''camera'''
        self.camera_rgb_list = []
        self.current_camera_index = 0

    def append(self, sensor, transform, callback):
        sensor_master = CarlaSensorMaster(sensor, transform, callback)
        self.sensor_list.append(sensor_master)
        self.sensor_dict[(sensor.type_id, sensor.attributes['role_name'])] = sensor_master

        if sensor_master.type_id == 'sensor.camera.rgb':
            self.camera_rgb_list.append(sensor_master)


    def get_camera(self):
        sensor_master = None
        try:
            sensor_master = self.camera_rgb_list[self.current_camera_index]
        except IndexError:
            pass
        return sensor_master
    def toggle_camera(self):
        self.current_camera_index = (self.current_camera_index + 1) % len(self.camera_rgb_list)


    def destroy(self):
        for sensor_master in self.sensor_dict.values():
            sensor_master.destroy()

    def __del__(self):
        self.destroy()

    def __iter__(self):
        for sensor_master in self.sensor_dict.values():
            yield sensor_master
    
    def __getitem__(self, key):
        if key in self.sensor_dict:
            return self.sensor_dict[key]
        else:
            debug(info='No sensor called '+ str(key), info_type='error')
            return None
    
    def __setitem__(self, key, value):
        if key in self.sensor_dict:
            self.sensor_dict[key] = value
            return True
        else:
            debug(info='No sensor called '+ str(key), info_type='error')
            return None


class CarlaSensorMaster(object):
    def __init__(self, sensor, transform, callback):
        self.sensor = sensor
        self.transform = transform
        self.raw_data, self.data = None, None

        self.type_id = sensor.type_id
        self.attributes = sensor.attributes

        if 'lidar' in sensor.type_id:
            self.frame_id = 'lidar/{}'.format(sensor.attributes['role_name'])
        elif 'camera' in sensor.type_id:
            self.frame_id = 'camera_rgb/{}'.format(sensor.attributes['role_name'])
        elif 'gnss' in sensor.type_id:
            self.frame_id = 'gnss/{}'.format(sensor.attributes['role_name'])
        elif 'imu' in sensor.type_id:
            self.frame_id = 'imu/{}'.format(sensor.attributes['role_name'])
        elif 'collision' in sensor.type_id:
            self.frame_id = 'collision/{}'.format(sensor.attributes['role_name'])

        weak_self = weakref.ref(self)
        if callback is not None:
            self.callback = lambda data: callback(weak_self, data)
        else:
            '''default callback'''
            if 'lidar' in sensor.type_id:
                self.callback = lambda data: CarlaSensorCallback.lidar(weak_self, data)
            elif 'camera' in sensor.type_id:
                self.callback = lambda data: CarlaSensorCallback.camera_rgb(weak_self, data)
            elif 'gnss' in sensor.type_id:
                self.callback = lambda data: CarlaSensorCallback.gnss(weak_self, data)
            elif 'imu' in sensor.type_id:
                self.callback = lambda data: CarlaSensorCallback.imu(weak_self, data)
            elif 'collision' in sensor.type_id:
                self.callback = lambda data: CarlaSensorCallback.collision(weak_self, data)

        if hasattr(sensor, 'listen'):
            self.sensor.listen(self.callback)

        return


    def get_transform(self):
        '''
            transform relative to parent actor
        '''
        return self.transform
    def get_world_transform(self):
        return self.sensor.get_transform()

    def get_raw_data(self):
        return self.raw_data
    def get_data(self):
        return self.data


    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()
