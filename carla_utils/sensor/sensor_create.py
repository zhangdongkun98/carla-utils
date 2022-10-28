from carla_utils import carla
SpawnActor = carla.command.SpawnActor


def create_sensor(core, vehicle, blueprint_library, param):
    world = core.world
    func_name = param['type_id'].replace('.', '_')
    func = getattr(CreateSensorBlueprint, func_name)
    bp = func(param, blueprint_library)
    sensor = world.spawn_actor(bp, param['transform'], attach_to=vehicle)
    return sensor

def create_sensor_command(core, vehicle, blueprint_library, param):
    world = core.world
    func_name = param['type_id'].replace('.', '_')
    func = getattr(CreateSensorBlueprint, func_name)
    bp = func(param, blueprint_library)
    cmd = SpawnActor(bp, param['transform'], vehicle)
    return cmd


class CreateSensorBlueprint(object):
    @staticmethod
    def sensor_camera_rgb(param, blueprint_library):
        bp = blueprint_library.find('sensor.camera.rgb')
        bp.set_attribute('role_name', str(param['role_name']))
        bp.set_attribute('image_size_x', str(param['image_size_x']))
        bp.set_attribute('image_size_y', str(param['image_size_y']))
        bp.set_attribute('fov', str(param['fov']))
        bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        # if bp.has_attribute('exposure_mode'):
        #     bp.set_attribute('exposure_mode', param['exposure_mode'])
        return bp

    @staticmethod
    def sensor_lidar_ray_cast(param, blueprint_library):
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        lidar_bp.set_attribute('role_name', str(param['role_name']))

        lidar_bp.set_attribute('channels', str(param['channels']))
        lidar_bp.set_attribute('range', str(param['range']))
        lidar_bp.set_attribute('points_per_second', str(param['points_per_second']))
        lidar_bp.set_attribute('rotation_frequency', str(param['rotation_frequency']))
        lidar_bp.set_attribute('upper_fov', str(param['upper_fov']))
        lidar_bp.set_attribute('lower_fov', str(param['lower_fov']))
        return lidar_bp
    
    @staticmethod
    def sensor_lidar_ray_cast_semantic(param, blueprint_library):
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        lidar_bp.set_attribute('role_name', str(param['role_name']))

        lidar_bp.set_attribute('channels', str(param['channels']))
        lidar_bp.set_attribute('range', str(param['range']))
        lidar_bp.set_attribute('points_per_second', str(param['points_per_second']))
        lidar_bp.set_attribute('rotation_frequency', str(param['rotation_frequency']))
        lidar_bp.set_attribute('upper_fov', str(param['upper_fov']))
        lidar_bp.set_attribute('lower_fov', str(param['lower_fov']))
        # lidar_bp.set_attribute('horizontal_fov', str(param['horizontal_fov']))
        return lidar_bp

    @staticmethod
    def sensor_other_imu(param, blueprint_library):
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('role_name', str(param['role_name']))
        imu_bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        return imu_bp

    @staticmethod
    def sensor_other_gnss(param, blueprint_library):
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_bp.set_attribute('role_name', str(param['role_name']))
        gnss_bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        return gnss_bp

    @staticmethod
    def sensor_other_collision(param, blueprint_library):
        collision_bp = blueprint_library.find('sensor.other.collision')
        collision_bp.set_attribute('role_name', str(param['role_name']))
        return collision_bp

    @staticmethod
    def sensor_camera_semantic_segmentation(param, blueprint_library):
        camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        camera_bp.set_attribute('role_name', str(param['role_name']))
        camera_bp.set_attribute('image_size_x', str(param['image_size_x']))
        camera_bp.set_attribute('image_size_y', str(param['image_size_y']))
        camera_bp.set_attribute('fov', str(param['fov']))
        camera_bp.set_attribute('sensor_tick', str(param['sensor_tick']))
        return camera_bp

    @staticmethod
    def sensor_other_lane_invasion(param, blueprint_library):
        bp = blueprint_library.find('sensor.other.lane_invasion')
        bp.set_attribute('role_name', str(param['role_name']))
        return bp

