# sensor_callback.py

class CarlaSensorCallback(object):
    @staticmethod
    def lidar(weak_self, data):
        # data: carla.LidarMeasurement
        self = weak_self()
        self.raw_data = data
        self.data = data

    @staticmethod
    def camera_rgb(weak_self, data):
        # data: carla.Image
        self = weak_self()
        self.raw_data = data
        self.data = data

    @staticmethod
    def gnss(weak_self, data):
        # data: carla.GNSSMeasurement
        self = weak_self()
        self.raw_data = data
        self.data = data

    @staticmethod
    def imu(weak_self, data):
        # data: carla.IMUMeasurement
        self = weak_self()
        self.raw_data = data
        self.data = data

    @staticmethod
    def collision(weak_self, data):
        # data: carla.CollisionEvent
        self = weak_self()
        self.raw_data = data
        self.data = data