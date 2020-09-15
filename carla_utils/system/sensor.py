
import carla
import numpy as np

def image_callback(weak_self, data):
    # data: carla.Image
    self = weak_self()
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8")) 
    array = np.reshape(array, (data.height, data.width, 4)) # RGBA format
    self.raw_data = data
    self.data = array
    
def lidar_callback(weak_self, data):
    # data: carla.LidarMeasurement
    self = weak_self()
    lidar_data = np.frombuffer(data.raw_data, dtype=np.float32).reshape([-1, 3])
    point_cloud = np.stack([-lidar_data[:,1], -lidar_data[:,0], -lidar_data[:,2]])
    mask = np.where((point_cloud[0] > 1.0)|(point_cloud[0] < -4.0)|(point_cloud[1] > 1.2)|(point_cloud[1] < -1.2))[0]
    point_cloud = point_cloud[:, mask]
    mask = np.where(point_cloud[2] > -1.95)[0]
    point_cloud = point_cloud[:, mask]
    self.raw_data = data
    self.data = point_cloud



sensor_param_dict = {
    'camera':{
        'img_length': 640,
        'img_width': 360,
        'image_size_x': 640,
        'image_size_y': 360,
        'role_name': 'front',
        'fov': 90,
        'fps': 30,
        'transform':carla.Transform(carla.Location(x=1.5, y=0.0, z=2.0)),
        'callback':image_callback,
    },
    'lidar':{
        'channels': 32,
        'rpm': 10,
        'pps': 172800,
        'range': 50,
        'lower_fov': -30,
        'upper_fov': 10,
        'sensor_tick': 0.45,
        'transform':carla.Transform(carla.Location(x=1.5, y=0.0, z=2.0)),
        'callback':lidar_callback,
    },

}