import carla

import numpy as np
from typing import AnyStr, List
import pickle
from collections import namedtuple

from .agent_base import BaseAgent


class AgentsRecorder(object):
    def __init__(self):
        self.records = dict()

    def record(self, timestamp : AnyStr, agents : List[BaseAgent]):
        """
        
        
        Args:
            timestamp: time.time()
            agents: list of BaseAgent
        
        Returns:
            
        """
        
        self.records[timestamp] = dict()
        for agent in agents:
            attributes = agent.vehicle.attributes
            attributes['role_name'] = pickle.loads(bytes(attributes['role_name'], encoding='utf-8'))
            agent_id = attributes['role_name'].vi
            bbx = agent.vehicle.bounding_box.extent
            x, y, z = bbx.x, bbx.y, bbx.z
            bbx = PicklableBoundingBox(x, y, z)
            self.records[timestamp][agent_id] = PicklableAgent(id=agent_id, state=agent.get_state(), attributes=attributes, bounding_box=bbx)
        return


    def save_to_disk(self, file_path):
        with open(file_path, 'wb') as f:
            pickle.dump(self.records, f)
        return


    @staticmethod
    def load_from_disk(file_path):
        record = None
        with open(file_path, 'rb') as f:
            record = pickle.load(f)
        return record



PicklableAgent = namedtuple('PicklableAgent', ('id', 'state', 'attributes', 'bounding_box'))

class PicklableAgent(object):
    def __init__(self, **kwargs):
        self.id = kwargs['id']
        self.state = kwargs['state']
        self.attributes = kwargs['attributes']
        self.bounding_box = kwargs['bounding_box']
    
    def get_transform(self):
        x, y, z = self.state.x, self.state.y, self.state.z
        theta = self.state.theta
        location = carla.Location(x, y, z)
        rotation = carla.Rotation(yaw=np.rad2deg(theta))
        return carla.Transform(location, rotation)



PicklableExtent = namedtuple('PicklableExtent', ('x', 'y', 'z'))
class PicklableBoundingBox(object):
    def __init__(self, x, y, z):
        self.location, self.rotation = None, None
        self.extent = PicklableExtent(x, y, z)

