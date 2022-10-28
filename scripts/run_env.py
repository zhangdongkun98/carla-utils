import rldev
import carla_utils as cu
from carla_utils import carla
from carla_utils.benchmark import scenario
from carla_utils import rl_template
from carla_utils.benchmark.scenario import ScenarioNoCrashBenchmark

import os
import time
import copy
import numpy as np
import matplotlib.pyplot as plt
import cv2
import torch



def generate_argparser():
    from carla_utils.utils import default_argparser
    argparser = default_argparser()

    argparser.add_argument('-d', dest='description', default='Nothing', help='[Method] description.')

    argparser.add_argument('--seed', default=0, type=int, help='seed.')
    argparser.add_argument('--num-episodes', default=20000, type=int, help='number of episodes.')
    
    argparser.add_argument('--evaluate', action='store_false', help='evaluate models (default: False)')
    return argparser



sensors_param_list = [
    {
        'type_id': 'sensor.other.collision',
        'role_name': 'default',
        'transform': carla.Transform(carla.Location(x=2.5, z=0.7)),
    },

    {
        'type_id': 'sensor.camera.rgb',
        'role_name': 'view',
        'image_size_x': 640,
        'image_size_y': 360,
        'fov': 120,
        'transform': carla.Transform(carla.Location(x=0, z=2.8), carla.Rotation(pitch=-5)),
    },
]




class ScenarioNoCrashBenchmark(scenario.ScenarioNoCrashBenchmark):
    mode_town = ['train', 'evaluate'][0]
    mode_weather = ['train', 'evaluate'][0]
    mode_traffic = ['empty', 'regular', 'dense'][-1]

    config_scenario = scenario.ScenarioNoCrashBenchmark.get_config_scenario(mode_town, mode_weather, mode_traffic)

    map_name = config_scenario.map_name
    weathers = config_scenario.weathers
    weather = config_scenario.weather
    num_vehicles = config_scenario.num_vehicles
    num_vehicles_min = config_scenario.num_vehicles_min
    num_vehicles_max = config_scenario.num_vehicles_max
    num_walkers = config_scenario.num_walkers


    time_tolerance = 200
    max_velocity = 8.0
    type_id = 'vehicle.tesla.model3'
    obstacle_type_id = 'vehicle.*'





class PerceptionImage(object):
    dim_state = cu.basic.Data(ego=1, route=40, obs=(320, 180))
    def __init__(self, config, **kwargs):
        self.config = config
        self.perp_gt_route = cu.perception.GroundTruthRoute(config, self.dim_state.route, perception_range=30)
        self.leading_range = 30.0
        return

    def run_step(self, step_reset, time_step, agents):
        self.step_reset, self.time_step = step_reset, time_step

        agent = agents[0]

        ego = self.get_state_ego(agent)
        route = self.perp_gt_route.run_step(agent)
        image = agent.sensors_master.get_camera().data[...,:-1]
        image = cv2.resize(image, self.dim_state.obs)
        image = image.astype(np.float32) /255

        current_transform = agent.get_transform()
        reference_waypoints, remaining_distance = agent.global_path.remaining_waypoints(current_transform)
        agent, distance = cu.agents.get_leading_agent_unsafe(agent, agents, reference_waypoints, max_distance=self.leading_range)
        if agent == None:
            distance = self.leading_range
        affordance = np.asarray([distance], dtype=np.float32) /self.leading_range

        return rldev.Data(ego=ego, route=route, obs=image, affordance=affordance)


    def get_state_ego(self, agent):
        state = np.array([
                agent.get_state().v /agent.max_velocity,
            ], dtype=np.float32)
        return state


    # def viz(self, data):
    #     image = data.obs
    #     save_dir = os.path.join(self.config.path_pack.output_path, str(self.step_reset))
    #     cu.system.mkdir(save_dir)
    #     cv2.imwrite(os.path.join(save_dir, '{}.png'.format(self.time_step)), (image*255).astype(np.uint8))
    #     return




class AgentLongitudinalControl(cu.BaseAgent):
    dim_action = 1
    def get_target(self, reference):
        velocity = (np.clip(reference[0].item(), -1,1) + 1) * self.max_velocity /2
        target = velocity
        return target


    def get_control(self, target):
        current_transform = self.get_transform()
        target_waypoint, curvature = self.global_path.target_waypoint(current_transform)

        velocity = target

        current_v = self.get_current_v()
        current_state = cu.cvt.CuaState.carla_transform(current_transform, v=current_v)
        target_state = cu.cvt.CuaState.carla_transform(target_waypoint.transform, v=velocity, k=curvature)
        control = self.controller.run_step(current_state, target_state)
        return control




class AgentListMaster(cu.AgentListMaster):
    Perception = PerceptionImage
    Agent = AgentLongitudinalControl

    dim_state = Perception.dim_state
    dim_action = Agent.dim_action

    def __init__(self, config, **kwargs):
        super().__init__(config)

        self.num_vehicles = config.num_vehicles
        self.dim_state = config.dim_state
        self.max_velocity = config.max_velocity
        self.perception_range = config.perception_range

        self.perp = self.Perception(config)


    def get_agent_type(self, learnable=True):
        if learnable:
            agent_type = self.Agent
        else:
            raise NotImplementedError
        return agent_type
    

    def perception(self, index, time_step):
        state = self.perp.run_step(index, time_step, self.agents + self.obstacles)
        return state.to_tensor().unsqueeze(0)



    def visualize(self, state: rldev.Data):
        image = state.obs.squeeze(0).numpy()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        plt.imshow(image)
        plt.pause(0.001)
        return




class EnvNoCrashBenchmark(rl_template.EnvSingleAgent):
    scenario_cls = ScenarioNoCrashBenchmark
    agents_master_cls = AgentListMaster
    recorder_cls = rl_template.PseudoRecorder

    sensors_params = sensors_param_list

    decision_frequency = 10
    control_frequency = 40

    perception_range = 50.0


    def reset(self):
        super().reset()
        self.state = self.agents_master.perception(self.step_reset, self.time_step)
        return self.state



    @torch.no_grad()
    def _step_train(self, action):
        self.time_step += 1

        ### state
        state = self.state
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
        self.agents_master.run_step(action)

        ### next_state
        next_state = self.agents_master.perception(self.step_reset, self.time_step)
        self.state = copy.copy(next_state)

        ### experience
        reward = torch.tensor([reward], dtype=torch.float32)
        done = torch.tensor([epoch_done], dtype=torch.float32)
        experience = rldev.Data(
            state=state, action=action, next_state=next_state, reward=reward,
            done=done,
        )
        return experience, epoch_done, epoch_info




    @property
    def settings(self):
        st = cu.default_settings(sync=True, render=True, dt=1/self.control_frequency)
        return st



def run_one_episode(env):
    env.reset()
    while True:
        action = env.action_space.sample()
        experience, epoch_done, info = env.step(action)
        if env.config.render:
            env.render()
        
        if epoch_done == True:
            env.on_episode_end()
            break
    return


if __name__ == "__main__":
    config = rldev.YamlConfig()
    args = generate_argparser().parse_args()
    config.update(args)

    mode = 'train'
    if config.evaluate == True:
        mode = 'evaluate'
        config.seed += 1
    rldev.setup_seed(config.seed)

    ### tensorboard log
    writer = rldev.create_dir(config, model_name='Demo', mode=mode)

    ### env
    Env = EnvNoCrashBenchmark
    env = Env(config, writer, mode=mode, env_index=-1)



    print('\n' + rldev.prefix(__name__) + 'env: ', rldev.get_class_name(env))
    print('\nconfig: \n', config)
    try:
        cu.destroy_all_actors(config.core)
        for _ in range(config.num_episodes):
            run_one_episode(env)
    finally:
        writer.close()
        env.destroy()
