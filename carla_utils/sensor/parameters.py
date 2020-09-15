
import numpy as np

from .coordinate_transformation import CoordinateTransformation, rotationMatrix3D



def intrinsicMatrix(fx, fy, u0, v0):
    K = np.array([  [fx, 0, u0],
                    [0, fy, v0],
                    [0,  0,  1] ])
    return K


class IntrinsicParams(object):
    def __init__(self, sensor):
        '''
        Args:
            sensor: carla.Sensor
        '''
        image_size_x = float(sensor.attributes['image_size_x'])
        image_size_y = float(sensor.attributes['image_size_y'])
        fov = eval(sensor.attributes['fov'])
        f = image_size_x /(2 * np.tan(fov * np.pi / 360))

        # [px]
        fx = f
        fy = f
        u0 = image_size_x / 2
        v0 = image_size_y / 2

        self.K = intrinsicMatrix(fx, fy, u0, v0)
        

class ExtrinsicParams(object):
    def __init__(self, sensor):
        '''
        Args:
            sensor: carla.Sensor
        '''

        # camera coordinate in world coordinate
        transform = sensor.get_transform()

        # [m]
        x = transform.location.x
        y = transform.location.y
        z = transform.location.z

        # [rad]
        roll = np.deg2rad(transform.rotation.roll)
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)

        # (coordinate) t: camera in world, R: camera to world
        self.t = np.array([[x, y, z]]).T
        self.R = rotationMatrix3D(roll, pitch, yaw)


class CameraParams(object):
    def __init__(self, intrinsic_params, extrinsic_params):
        '''
        Args:
            intrinsic_params: IntrinsicParams
            extrinsic_params: ExtrinsicParams
        '''
        self.K = intrinsic_params.K
        self.t = extrinsic_params.t
        self.R = extrinsic_params.R
        


