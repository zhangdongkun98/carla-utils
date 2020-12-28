import carla
ApplyVehicleControl = carla.command.ApplyVehicleControl
ApplyTransform = carla.command.ApplyTransform

from .agent_base import BaseAgent
from ..system import Clock
from ..augment import CollisionCheck
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

        self.agents, self.agents_learnable = [], []
        self.run_step = self._run_step_pseudo if self.pseudo else self._run_step_real


    def register(self, agent : BaseAgent, learnable=True):
        assert self.pseudo == agent.pseudo
        self.agents.append(agent)
        if learnable: self.agents_learnable.append(agent)
    
    def remove(self, agent):
        if agent in self.agents_learnable: self.agents_learnable.remove(agent)
        if agent in self.agents: self.agents.remove(agent)

    
    def destroy(self):
        batch = []
        for agent in self.agents: batch.extend(agent.destroy_commands())
        self.client.apply_batch_sync(batch)
        tick_world(self.world)
        self.agents, self.agents_learnable = [], []
    

    def _run_step_real(self, references):
        target_v_list = [agent.get_target_v(reference) for agent, reference in zip(self.agents, references)]
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            batch = []
            for agent, target_v in zip(self.agents, target_v_list):
                control = agent.get_control(target_v)
                batch.append(ApplyVehicleControl(agent.vehicle, control))
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
    
    def _run_step_pseudo_v2(self, references):
        [agent.tick_transform() for agent in self.agents]
        target_v_list = [agent.get_target_v(reference) for agent, reference in zip(self.agents, references)]
        dist = [0.0 for _ in range(len(self.agents))]
        for _ in range(self.skip_num):
            self.clock.tick_begin()
            for i, (agent, target_v) in enumerate(zip(self.agents, target_v_list)):
                dist[i] += agent.next_transform(target_v)
            if not self.fast: self.clock.tick_end()
        batch = [ApplyTransform(agent.vehicle, agent.get_transform_pesudo()) for agent in self.agents]
        self.client.apply_batch_sync(batch)

        print('dist: ', dist)
        return
    

    def check_collision(self):
        num_agents = len(self.agents)
        collisions = [False] * num_agents
        for i, agent in enumerate(self.agents):
            if collisions[i] == True: continue
            for j, other_agent in enumerate(self.agents):
                if agent.id == other_agent.id: continue
                if CollisionCheck.d2(agent.vehicle, other_agent.vehicle):
                    collisions[i] = True
                    collisions[j] = True
        return collisions

        




