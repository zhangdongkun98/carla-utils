import rldev
from carla_utils import carla

import time
from os.path import join
from abc import ABC, abstractmethod
import threading
import numpy as np
import copy

import torch

from ..system import mkdir, Clock
from ..world_map import Core
from ..world_map import default_settings
from ..agents import AgentListMaster

from ..basic import Data, prefix
from ..basic import Data as Experience
from ..basic import YamlConfig

from .scenario import ScenarioSingleAgent
from .recorder import Recorder
from carla_utils.rl_template.reward_function import RewardFunction


class EnvSingleAgent(ABC):
    """
    Suppose:
        1. Obstacles have no sensors.
        2. One env has its own config.

    """

    modes = ['train', 'evaluate', 'real']

    scenario_cls = ScenarioSingleAgent
    recorder_cls = Recorder
    agents_master_cls = AgentListMaster
    reward_function_cls = RewardFunction

    sensors_params = []

    decision_frequency = None
    control_frequency = None

    perception_range = None


    def __init__(self, config: YamlConfig, writer: rldev.Writer, mode, env_index=-100):
        ### set parameters
        self.set_parameters(config, mode, env_index)

        self.config, self.path_pack = config, config.path_pack
        self.writer = writer
        assert mode in self.modes
        self.mode = mode
        self.env_index = env_index
        self.env_name = 'env' + str(env_index)
        self.output_dir = join(self.path_pack.output_path, self.env_name)
        mkdir(self.output_dir)

        self.clock = Clock(self.control_frequency)
        self.clock_decision = Clock(self.decision_frequency)
        self.step_reset = 0

        ### default class
        self.recorder = config.get('recorder_cls', self.recorder_cls)(self.output_dir)
        self.reward_function = config.get('reward_function_cls', self.reward_function_cls)(config)
        self.agents_master = None

        ### client
        scenario_cls = config.get('scenario_cls', self.scenario_cls)
        init_map_name = scenario_cls.map_name
        if init_map_name == None:
            raise RuntimeError('{} has no map'.format(scenario_cls))
        self.core = Core(config, init_map_name, settings=self.settings, use_tm=True)
        self.client, self.world, self.town_map = self.core.client, self.core.world, self.core.town_map
        self.traffic_manager = self.core.traffic_manager
        self.scenario: ScenarioSingleAgent = scenario_cls(config)

        agents_master_cls = self.config.get('agents_master_cls', self.agents_master_cls)
        self.dim_state = agents_master_cls.dim_state
        self.dim_action = agents_master_cls.dim_action

        self.init()
        return
    


    def reset(self):
        ### save record and destroy agents
        self._destroy_agents()

        ### env param
        self.step_reset += 1
        self.time_step = 0

        ### scenario param
        map_name, weather = self.scenario.get_map(), self.scenario.get_weather()
        self.core.load_map(map_name, weather)
        self.scenario.reset()
        self.time_tolerance = self.scenario.time_tolerance
        self.map_name = self.scenario.map_name
        self.max_velocity = self.scenario.max_velocity
        self.num_vehicles = self.scenario.num_vehicles
        self.num_vehicles_max = self.scenario.num_vehicles_max

        ### record scenario
        self.recorder.record_town_map(self.scenario)

        ### register agents
        agents_master_cls = self.config.get('agents_master_cls', self.agents_master_cls)
        agents_master_kwargs = rldev.Data(
            time_tolerance=self.time_tolerance,
            num_vehicles=self.num_vehicles,
        ).to_dict()
        self.agents_master: AgentListMaster = agents_master_cls(self.config, **agents_master_kwargs)
        self.scenario.register_agents(self.step_reset, self.agents_master, self.sensors_params)
        if self.mode == 'real':
            time.sleep(1/self.decision_frequency)

        ### recorder
        self.recorder.record_scenario(self.config, self.step_reset, self.scenario)

        ### callback
        self.on_episode_start()
        return



    @torch.no_grad()
    def _step_train(self, method):
        timestamp = str(time.time())
        self.time_step += 1

        ### state
        if self.time_step == 1:
            state = self.agents_master.perception(self.step_reset, self.time_step)
        else:
            state = self.state
        ### action
        action = method.select_action(state).cpu()
        ### reward
        epoch_info = self._check_epoch()
        reward = self.reward_function.run_step(state, action, self.agents_master, epoch_info)
        epoch_done = epoch_info.done
        
        ### record
        self.recorder.record_agents(self.time_step, self.agents_master, epoch_info)
        self.recorder.record_experience(self.time_step, self.agents_master, action)
        
        ### callback
        self.on_episode_step(reward, epoch_info)

        ### step
        self.agents_master.run_step( action.action if isinstance(action, Data) else action )

        ### next_state
        next_state = self.agents_master.perception(self.step_reset, timestamp=-1)
        self.state = copy.copy(next_state)

        ### experience
        reward = torch.tensor([reward], dtype=torch.float32)
        done = torch.tensor([epoch_done], dtype=torch.float32)
        if done == True:
            self.on_episode_end()
        experience = Experience(
            state=state, action=action, next_state=next_state, reward=reward,
            done=done, timestamp=timestamp,
        )
        return experience, epoch_done, epoch_info
    

    @torch.no_grad()
    def _step_evaluate(self, *args):
        return self._step_train(*args)


    @torch.no_grad()
    def _step_real(self, method):
        self.clock_decision.tick_begin()
        timestamp = str(time.time())
        self.time_step += 1

        ### state
        state = self.agents_master.perception(self.step_reset, self.time_step)
        ### reward
        action = method.select_action(state).cpu()
        reward, epoch_info = self._calculate_reward(state, action)
        epoch_done = epoch_info.done

        ### step
        self.agents_master.run_step( action.action if isinstance(action, Data) else action )
        
        ### next_state
        next_state = self.agents_master.perception(self.time_step, self.agents_master)

        ### experience
        reward = torch.tensor([reward], dtype=torch.float32)
        done = torch.tensor([epoch_done], dtype=torch.float32)
        if done == True:
            self.on_episode_end()
        experience = Experience(
            state=state, action=action, next_state=next_state, reward=reward,
            done=done, timestamp=timestamp,
        )
        self.clock_decision.tick_end()
        return experience, epoch_done, epoch_info


    def render(self):
        self.agents_master.visualize(self.state)



    @property
    def action_space(self):
        return SingleAgentBoxSpace(-1.0, 1.0, shape=(1, self.dim_action), dtype=np.float32)



    @classmethod
    def set_parameters(cls, config: YamlConfig, mode, env_index=-100):
        config.set('mode', mode)
        config.set('env_index', env_index)
        config.set('decision_frequency', cls.decision_frequency)
        config.set('control_frequency', cls.control_frequency)
        config.set('perception_range', cls.perception_range)

        agents_master_cls = config.get('agents_master_cls', cls.agents_master_cls)
        config.set('dim_state', agents_master_cls.dim_state)
        config.set('dim_action', agents_master_cls.dim_action)

        if mode == 'real':
            real = True
        else:
            real = False
        config.set('real', real)
        return


    @property
    def settings(self):
        if self.mode == 'real':
            st = default_settings(sync=False, render=True, dt=0.0)
        else:
            st = default_settings(sync=True, render=False, dt=1/self.control_frequency)
        return st


    def init(self):
        if self.mode == 'real':
            self.step = self._step_real
            print(prefix(self) + ' Env is in real mode.')

            self.target = None
            self.thread_stoped = False
            self.thread_control = threading.Thread(target=self._run_control, args=(lambda: self.thread_stoped,))
            self.thread_control.start()

        elif self.mode == 'train':
            self.step = self._step_train
        elif self.mode == 'evaluate':
            self.step = self._step_evaluate
        else:
            raise NotImplementedError('Unknown Mode.')
        return


    def _destroy_agents(self):
        if self.agents_master != None:
            self.recorder.save_to_disk(self.step_reset, self.client)
            self.recorder.clear()
            self.agents_master.destroy()
        self.agents_master = None
        return

    def destroy(self):
        if self.mode == 'real':
            self.thread_stoped = True
            self.thread_control.join()
        self._destroy_agents()
        del self.town_map, self.world, self.client
        self.core.kill()
        del self.core


    def _check_epoch(self):
        '''check if collision, timeout, success, dones'''
        collisions = [a.check_collision() for a in self.agents_master.agents]
        timeouts = [a.check_timeout(self.time_tolerance) for a in self.agents_master.agents]
        # timeouts = [self.check_timeout()]
        epoch_done = collisions[0] | timeouts[0]
        epoch_info = Data(done=epoch_done, c=collisions[0], t=timeouts[0])
        return epoch_info


    def check_timeout(self):
        return self.time_step >= self.time_tolerance



    def on_episode_start(self):
        return
    def on_episode_step(self, *args, **kwargs):
        return
    def on_episode_end(self):
        return



    def _run_control(self, stop):
        while True:
            if self.agents_master == None:
                time.sleep(0.01)
                continue

            self.clock.tick_begin()

            self.agents_master.run_control()

            if stop(): break
            self.clock.tick_end()
        print(prefix(self) + 'Exit control thread.')
        return






class SingleAgentBoxSpace(object):
    def __init__(self, low, high, shape, dtype):
        self.low = low
        self.high = high
        self.shape = shape
        self.dtype = dtype

    def sample(self):
        return np.random.uniform(self.low,self.high, size=self.shape).astype(self.dtype)

