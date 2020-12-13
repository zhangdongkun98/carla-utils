import carla
ApplyVehicleControl = carla.command.ApplyVehicleControl
ApplyTransform = carla.command.ApplyTransform

from .agent_base import BaseAgent
from ..system import Clock
from ..world_map import tick_world


class AgentListMaster(object):
    def __init__(self, config, client, world):
        """
        
        
        Args:
            config: contains pseudo, fast
        
        Returns:
        -------
        """
        
        self.config = config
        self.client, self.world = client, world

        self.pseudo, self.fast = config.pseudo, config.fast

        self.decision_frequency = config.decision_frequency
        self.control_frequency = config.control_frequency
        self.skip_num = self.control_frequency // self.decision_frequency
        assert self.control_frequency % self.decision_frequency == 0

        self.clock = Clock(self.control_frequency)

        self.agents, self.vehicles = [], []
        self.num_agents = 0
        self.vehicle_ids = []
        self.run_step = self._run_step_pseudo if self.pseudo else self._run_step_real


    def register(self, agent : BaseAgent):
        assert self.pseudo == agent.pseudo
        self.agents.append(agent)
        self.vehicles.append(agent.vehicle)
        self.vehicle_ids.append(agent.vehicle.id)
        self.num_agents += 1
    
    def destroy(self):
        batch = []
        for agent in self.agents: batch.extend(agent.destroy_commands())
        self.client.apply_batch_sync(batch)
        tick_world(self.world)
    

    def _run_step_real(self, references):
        target_v_list = [agent.get_target_v(reference) for agent, reference in zip(self.agents, references)]
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            batch = []
            for agent, vehicle, target_v in zip(self.agents, self.vehicles, target_v_list):
                control = agent.get_control(target_v)
                batch.append(ApplyVehicleControl(vehicle, control))
            self.client.apply_batch_sync(batch)
            self.clock.tick_end()
        return
    
    def _run_step_pseudo(self, references):
        [agent.tick_transform() for agent in self.agents]
        target_v_list = [agent.get_target_v(reference) for agent, reference in zip(self.agents, references)]
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            for agent, target_v in zip(self.agents, target_v_list):
                agent.next_transform(target_v)
            if not self.fast: self.clock.tick_end()
        batch = [ApplyTransform(agent.vehicle, agent.get_transform_pesudo()) for agent in self.agents]
        self.client.apply_batch_sync(batch)
        return




