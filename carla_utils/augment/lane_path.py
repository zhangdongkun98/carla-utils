
import time
import numpy as np


class LanePath(object):
    def __init__(self, frame_id, time_stamp, lane_id, **kwargs):
        # default params
        self.frame_id = frame_id
        self.time_stamp = time_stamp
        self.size = 0
        self.waypoints = []
        self.steps = []

        self.lane_id = lane_id


    def append(self, step, waypoint):
        self.steps.append(step)
        self.waypoints.append(waypoint)
        self.size += 1