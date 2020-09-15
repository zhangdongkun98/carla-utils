# carla_sensor.py

import numpy as np
import weakref
import copy
import time

from ..system import Singleton, debug
from .sensor_callback import CarlaSensorCallback



def createSensoListMaster(world, vehicle, sensors_param_list):
    sensor_list_master = CarlaSensorListMaster(world, vehicle)
    for sensor_param_dict in sensors_param_list:
        callback = sensor_param_dict['callback']
        sensor = add_sensor(world, vehicle, sensor_param_dict)
        sensor_list_master.append(sensor, sensor_param_dict['transform'], callback)
    return sensor_list_master


def add_sensor(world, vehicle, sensor_param_dict):
    sensor = None
    blueprint = world.get_blueprint_library()
    if sensor_param_dict['type_id'] == 'sensor.camera.rgb':
        sensor = CreateSensor.add_camera(world, blueprint, vehicle, sensor_param_dict)
    elif sensor_param_dict['type_id'] == 'sensor.lidar.ray_cast':
        sensor = CreateSensor.add_lidar(world, blueprint, vehicle, sensor_param_dict)
    elif sensor_param_dict['type_id'] == 'sensor.other.imu':
        sensor = CreateSensor.add_imu(world, blueprint, vehicle, sensor_param_dict)
    elif sensor_param_dict['type_id'] == 'sensor.other.gnss':
        sensor = CreateSensor.add_gnss(world, blueprint, vehicle, sensor_param_dict)
    elif sensor_param_dict['type_id'] == 'sensor.other.collision':
        sensor = CreateSensor.add_coliision(world, blueprint, vehicle, sensor_param_dict)
    return sensor


class CreateSensor(object):
    @staticmethod
    def add_camera(world, blueprint, vehicle, config):
        camera_bp = blueprint.find('sensor.camera.rgb')
        camera_bp.set_attribute('role_name', str(config['role_name']))
        camera_bp.set_attribute('image_size_x', str(config['image_size_x']))
        camera_bp.set_attribute('image_size_y', str(config['image_size_y']))
        camera_bp.set_attribute('fov', str(config['fov']))
        camera_bp.set_attribute('sensor_tick', str(1./config['fps']))
        camera = world.spawn_actor(camera_bp, config['transform'], attach_to=vehicle)
        return camera

    @staticmethod
    def add_lidar(world, blueprint, vehicle, config):
        lidar_bp = blueprint.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('role_name', str(config['role_name']))
        lidar_bp.set_attribute('channels', str(config['channels']))
        lidar_bp.set_attribute('rotation_frequency', str(config['rpm']))
        lidar_bp.set_attribute('points_per_second', str(config['pps']))
        lidar_bp.set_attribute('sensor_tick', str(config['sensor_tick']))
        lidar_bp.set_attribute('range', str(config['range']))
        lidar_bp.set_attribute('lower_fov', str(config['lower_fov']))
        lidar_bp.set_attribute('upper_fov', str(config['upper_fov']))
        lidar = world.spawn_actor(lidar_bp, config['transform'], attach_to=vehicle)
        return lidar
    
    @staticmethod
    def add_imu(world, blueprint, vehicle, config):
        imu_bp = blueprint.find('sensor.other.imu')
        imu_bp.set_attribute('role_name', str(config['role_name']))
        imu_bp.set_attribute('sensor_tick', str(1./config['fps']))
        imu = world.spawn_actor(imu_bp, config['transform'], attach_to=vehicle)
        return imu

    @staticmethod
    def add_gnss(world, blueprint, vehicle, config):
        gnss_bp = blueprint.find('sensor.other.gnss')
        gnss_bp.set_attribute('role_name', str(config['role_name']))
        gnss_bp.set_attribute('sensor_tick', str(1./config['fps']))
        gnss = world.spawn_actor(gnss_bp, config['transform'], attach_to=vehicle)
        return gnss

    @staticmethod
    def add_coliision(world, blueprint, vehicle, config):
        coliision_bp = blueprint.find('sensor.other.collision')
        coliision_bp.set_attribute('role_name', str(config['role_name']))
        coliision = world.spawn_actor(coliision_bp, config['transform'], attach_to=vehicle)
        return coliision


class CarlaSensorListMaster(Singleton):
    def __init__(self, world, vehicle):
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
            
        pass


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
        self.sensor.destroy()
