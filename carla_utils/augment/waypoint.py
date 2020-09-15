
import numpy as np


class Waypoint(object):
    def __init__(self, frame_id, time_stamp, carla_waypoint, **kwargs):
        # default params
        self.frame_id = frame_id
        self.time_stamp = time_stamp

        self.x = carla_waypoint.transform.location.x
        self.y = carla_waypoint.transform.location.y
        self.z = carla_waypoint.transform.location.z
        self.theta = np.deg2rad(carla_waypoint.transform.rotation.yaw)
        self.lane_id = 0
        self.step = 0
        self.reference = False

        # for visualization
        self.roll_rad = np.deg2rad(carla_waypoint.transform.rotation.roll)
        self.pitch_rad= np.deg2rad(carla_waypoint.transform.rotation.pitch)
        self.yaw_rad  = np.deg2rad(carla_waypoint.transform.rotation.yaw)

        # parameters overload
        if 'lane_id' in kwargs:
            self.lane_id = kwargs['lane_id']
        if 'step' in kwargs:
            self.step = kwargs['step']
        if 'reference' in kwargs:
            self.reference = kwargs['reference']


    def distance_xyz(self, waypoint):
        dx = self.x - waypoint.x
        dy = self.y - waypoint.y
        dz = self.z - waypoint.z
        return np.sqrt(dx**2 + dy**2 + dz**2)
    def distance_xy(self, waypoint):
        dx = self.x - waypoint.x
        dy = self.y - waypoint.y
        return np.sqrt(dx**2 + dy**2)

