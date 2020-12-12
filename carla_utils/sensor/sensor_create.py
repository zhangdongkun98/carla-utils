import carla
SpawnActor = carla.command.SpawnActor


def create_sensor(world, vehicle, blueprint_library, config):
    func_name = config['type_id'].replace('.', '_')
    func = getattr(CreateSensorBlueprint, func_name)
    bp = func(world, vehicle, blueprint_library, config)
    sensor = world.spawn_actor(bp, config['transform'], attach_to=vehicle)
    return sensor

def create_sensor_command(world, vehicle, blueprint_library, config):
    func_name = config['type_id'].replace('.', '_')
    func = getattr(CreateSensorBlueprint, func_name)
    bp = func(world, vehicle, blueprint_library, config)
    cmd = SpawnActor(bp, config['transform'], vehicle)
    return cmd


class CreateSensorBlueprint(object):
    @staticmethod
    def sensor_camera_rgb(world, vehicle, blueprint_library, config):
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('role_name', str(config['role_name']))
        camera_bp.set_attribute('image_size_x', str(config['image_size_x']))
        camera_bp.set_attribute('image_size_y', str(config['image_size_y']))
        camera_bp.set_attribute('fov', str(config['fov']))
        camera_bp.set_attribute('sensor_tick', str(config['sensor_tick']))
        return camera_bp

    @staticmethod
    def sensor_lidar_ray_cast(world, vehicle, blueprint_library, config):
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('role_name', str(config['role_name']))
        lidar_bp.set_attribute('channels', str(config['channels']))
        lidar_bp.set_attribute('rotation_frequency', str(config['rpm']))
        lidar_bp.set_attribute('points_per_second', str(config['pps']))
        lidar_bp.set_attribute('sensor_tick', str(config['sensor_tick']))
        lidar_bp.set_attribute('range', str(config['range']))
        lidar_bp.set_attribute('lower_fov', str(config['lower_fov']))
        lidar_bp.set_attribute('upper_fov', str(config['upper_fov']))
        return lidar_bp
    
    @staticmethod
    def sensor_other_imu(world, vehicle, blueprint_library, config):
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('role_name', str(config['role_name']))
        imu_bp.set_attribute('sensor_tick', str(config['sensor_tick']))
        return imu_bp

    @staticmethod
    def sensor_other_gnss(world, vehicle, blueprint_library, config):
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_bp.set_attribute('role_name', str(config['role_name']))
        gnss_bp.set_attribute('sensor_tick', str(config['sensor_tick']))
        return gnss_bp

    @staticmethod
    def sensor_other_collision(world, vehicle, blueprint_library, config):
        collision_bp = blueprint_library.find('sensor.other.collision')
        collision_bp.set_attribute('role_name', str(config['role_name']))
        return collision_bp

