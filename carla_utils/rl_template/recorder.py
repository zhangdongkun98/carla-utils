import rldev
from carla_utils import carla

import numpy as np
from typing import List, Any
import pickle
import os
from os.path import join

from ..basic import Data, YamlConfig
from ..world_map import Role, get_topology
from ..augment import GlobalPath
from ..agents import AgentListMaster, BaseAgent

from .scenario import ScenarioSingleAgent, ScenarioRandomization


class Recorder(object):
    def __init__(self, dir_path):
        self.dir_path = dir_path
        self.records = dict()


    def record_town_map(self, scenario: ScenarioSingleAgent):
        file_path = join(self.dir_path, scenario.map_name + '.txt')
        if not os.path.isfile(file_path):
            with open(file_path, 'wb') as f:
                pickle.dump(PicklableTownMap(scenario.town_map), f)
        return

    def record_scenario(self, config: YamlConfig, step_reset, scenario: ScenarioSingleAgent):
        params = rldev.get_class_parameters(scenario.__class__)
        params.pop('__doc__')
        params.pop('__module__')
        self.records['scenario'] = {
            'frequency': config.decision_frequency,
        }
        self.records['scenario'].update(params)
        config.core.client.start_recorder(join(self.dir_path, 'recording_{}.log'.format(step_reset)))

    def record_agents(self, timestamp, agents_master: AgentListMaster, epoch_info: Data):
        """
        
        Args:
            timestamp: time.time()
            agents: list of BaseAgent and BaseAgentObstacle
        
        Returns:
            
        """

        for agent in agents_master.agents:
            agent_key = 'agent' + '_' + agent.role_name.atype.name + '_' + str(agent.vi)
            if self.records.get(agent_key) == None:
                self.records[agent_key] = dict()
            self.records[agent_key][timestamp] = Data(agent=PicklableAgent(agent))
        for obstacle in agents_master.obstacles:
            obstacle_key = 'obstacle' + '_' + obstacle.role_name.atype.name + '_' + str(obstacle.vi)
            if self.records.get(obstacle_key) == None:
                self.records[obstacle_key] = dict()
            self.records[obstacle_key][timestamp] = Data(agent=PicklableAgent(obstacle))

        if epoch_info.done:
            for agent in agents_master.agents:
                agent_key = 'agent' + '_' + agent.role_name.atype.name + '_' + str(agent.vi)
                global_path = PicklableGlobalPath(agent.global_path)
                for t, picklable_agent in self.records[agent_key].items():
                    picklable_agent.global_path = global_path

        return

    def record_experience(self, timestamp, agents_master: AgentListMaster, actions):
        for agent, action in zip(agents_master.agents, actions):
            agent_key = 'agent' + '_' + agent.role_name.atype.name + '_' + str(agent.vi)
            self.records[agent_key][timestamp].update(action=action)
        return



    def save_to_disk(self, index, client):
        file_path = join(self.dir_path, str(index) + '.txt')
        with open(file_path, 'wb') as f:
            pickle.dump(self.records, f)
        client.stop_recorder()
        return

    def clear(self):
        del self.records
        self.records = dict()


    @staticmethod
    def load_from_disk(file_path):
        record = None
        with open(file_path, 'rb') as f:
            record = pickle.load(f)
        return record



class PseudoRecorder(object):
    def __init__(self, *args, **kwargs):
        return

    def record_town_map(self, *args, **kwargs):
        return
    def record_scenario(self, *args, **kwargs):
        return
    def record_agents(self, *args, **kwargs):
        return
    def record_experience(self, *args, **kwargs):
        return
    def save_to_disk(self, *args, **kwargs):
        return
    def clear(self, *args, **kwargs):
        return



# =============================================================================
# -- Picklable  ---------------------------------------------------------------
# =============================================================================


class PicklableAgent(object):
    def __init__(self, agent: BaseAgent):
        self.id = agent.vi
        self.vi = agent.vi
        self.state = agent.get_state()

        attributes = agent.vehicle.attributes
        attributes['role_name'] = agent.role_name
        self.attributes = attributes
        self.type_id = agent.vehicle.type_id

        bbx = agent.vehicle.bounding_box.extent
        x, y, z = bbx.x, bbx.y, bbx.z
        bbx = PicklableBoundingBox(x, y, z)
        self.bounding_box = bbx

        self.max_velocity = agent.max_velocity if hasattr(agent, 'max_velocity') else None
        self.global_path = None

        ### vehicle state
        self.vehicle_location = PicklableLocation(agent.vehicle.get_location())
        self.vehicle_transform = PicklableTransform(agent.vehicle.get_transform())
        self.vehicle_velocity = PicklableVector3D(agent.vehicle.get_velocity())
        self.vehicle_angular_velocity = PicklableVector3D(agent.vehicle.get_angular_velocity())
        self.vehicle_acceleration = PicklableVector3D(agent.vehicle.get_acceleration())


    def get_transform(self):
        x, y, z = self.state.x, self.state.y, self.state.z
        theta = self.state.theta
        location = carla.Location(x, y, z)
        rotation = carla.Rotation(yaw=np.rad2deg(theta))
        return carla.Transform(location, rotation)

    def get_state(self):
        return self.state
    

    # ==============================
    # -- vehicle functions ---------
    # ==============================
    def get_vehicle_location(self):
        x, y, z = self.vehicle_location.x, self.vehicle_location.y, self.vehicle_location.z
        return carla.Location(x, y, z)
    
    def get_vehicle_transform(self):
        x, y, z = self.vehicle_transform.location.x, self.vehicle_transform.location.y, self.vehicle_transform.location.z
        roll, pitch, yaw = self.vehicle_transform.rotation.roll, self.vehicle_transform.rotation.pitch, self.vehicle_transform.rotation.yaw
        location = carla.Location(x, y, z)
        rotation = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
        return carla.Transform(location, rotation)
    
    def get_vehicle_velocity(self):
        x, y, z = self.vehicle_velocity.x, self.vehicle_velocity.y, self.vehicle_velocity.z
        return carla.Vector3D(x, y, z)
    
    def get_vehicle_angular_velocity(self):
        x, y, z = self.vehicle_angular_velocity.x, self.vehicle_angular_velocity.y, self.vehicle_angular_velocity.z
        return carla.Vector3D(x, y, z)





class PicklableBoundingBox(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.extent = self



class PicklableGlobalPath(object):
    def __init__(self, global_path: GlobalPath):
        self.route = [(PicklableWaypoint(wp), option) for wp, option in global_path.route]
        self.carla_waypoints = [PicklableWaypoint(wp) for wp in global_path.carla_waypoints]
        self.options = global_path.options
        self.x = global_path.x
        self.y = global_path.y
        self.z = global_path.z
        self.theta = global_path.theta
        self.curvatures = global_path.curvatures
        self.distances = global_path.distances
        self.sampling_resolution = global_path.sampling_resolution
        self._max_coverage = 0

    def __len__(self):
        return len(self.carla_waypoints)

    def _step_coverage(self, current_transform):
        return GlobalPath._step_coverage(self, current_transform)

    def remaining_waypoints(self, current_transform):
        return GlobalPath.remaining_waypoints(self, current_transform)



class PicklableWaypoint(object):
    def __init__(self, waypoint: carla.Waypoint):
        self.transform = PicklableTransform(waypoint.transform)

    def get_waypoint(self, town_map: carla.Map, project_to_road=True):
        x, y, z = self.transform.location.x, self.transform.location.y, self.transform.location.z
        location = carla.Location(x, y, z)
        return town_map.get_waypoint(location, project_to_road=project_to_road)

    def __str__(self):
        res = ''
        for (key, value) in self.__dict__.items():
            res += key + '=' + str(value) + ', '
        return self.__class__.__name__ + '({})'.format(res[:-2])

    def __repr__(self):
        return str(self)


class PicklableTransform(object):
    def __init__(self, transform: carla.Transform):
        self.location = PicklableLocation(transform.location)
        self.rotation = PicklableRotation(transform.rotation)

    def __str__(self):
        res = ''
        for (key, value) in self.__dict__.items():
            res += key + '=' + str(value) + ', '
        return self.__class__.__name__ + '({})'.format(res[:-2])

    def __repr__(self):
        return str(self)


class PicklableLocation(object):
    def __init__(self, location: carla.Location):
        self.x = location.x
        self.y = location.y
        self.z = location.z

    def distance(self, loc):
        dx = self.x - loc.x
        dy = self.y - loc.y
        dz = self.z - loc.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def __str__(self):
        res = ''
        for (key, value) in self.__dict__.items():
            res += key + '=' + str(value) + ', '
        return self.__class__.__name__ + '({})'.format(res[:-2])

    def __repr__(self):
        return str(self)

class PicklableRotation(object):
    def __init__(self, rotation: carla.Rotation):
        self.roll = rotation.roll
        self.pitch = rotation.pitch
        self.yaw = rotation.yaw

    def __str__(self):
        res = ''
        for (key, value) in self.__dict__.items():
            res += key + '=' + str(value) + ', '
        return self.__class__.__name__ + '({})'.format(res[:-2])

    def __repr__(self):
        return str(self)


class PicklableTownMap(object):
    def __init__(self, town_map):
        self.name = town_map.name
        self.cua_waypoints = [PicklableWaypoint(wp) for wp in town_map.generate_waypoints(0.1)]
        self.opendrive_content = town_map.to_opendrive()
        self.topology_origin = [(PicklableWaypoint(start), PicklableWaypoint(end)) for (start, end) in town_map.get_topology()]
        self.topology = [t.info for t in get_topology(town_map, sampling_resolution=2.0)]


    def generate_waypoints(self, _):
        return self.cua_waypoints

    def to_opendrive(self):
        return self.opendrive_content

    def get_topology(self):
        return self.topology_origin



class PicklableVector3D(object):
    def __init__(self, vector: carla.Vector3D):
        self.x = vector.x
        self.y = vector.y
        self.z = vector.z




class PicklableScenarioRandomization(object):
    def __init__(self, scenario_randomization: ScenarioRandomization):
        self.num_vehicles = scenario_randomization.num_vehicles
        self.spawn_points = [PicklableLocation(spawn_point) for spawn_point in scenario_randomization.spawn_points]
        self.global_paths = [PicklableGlobalPath(global_path) for global_path in scenario_randomization.global_paths]
        return


