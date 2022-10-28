import carla_utils as cu
from carla_utils import carla

import numpy as np
import os, sys
from os.path import join
import threading
from tqdm import tqdm


from carla_utils.basic import YamlConfig
from carla_utils.rl_template.recorder import PicklableTownMap
from carla_utils.viz import plt
from carla_utils.viz import open3d


class Vopen3d(open3d.Visualizer):
    def connect_to_server(self):
        self.world = None
        self.town_map = self.config.town_map

class Vplt(plt.Visualizer):
    def connect_to_server(self):
        self.world = None
        self.town_map: PicklableTownMap = self.config.town_map

    def run_once(self):
        xlim, ylim = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.clear()
        self.ax.set_title(self.title_name)
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        if not hasattr(self, 'background_topology'):
            self.background_topology = self.town_map.topology
        for t in self.background_topology:
            line = self.ax.plot(t.x, t.y)[0]
        
        self.update_vis()
        return



class ReplayRecord(object):
    def __init__(self, config):
        self.index = config.index
        file_path = join(config.dir, str(config.index) + '.txt')
        self._record = cu.rl_template.Recorder.load_from_disk(file_path)
        
        scenario = self._record.pop('scenario')

        map_path = join(config.dir, scenario['map_name'] + '.txt')
        town_map = cu.rl_template.Recorder.load_from_disk(map_path)
        config.set('town_map', town_map)

        self.vv = config.viz_type(config)
        if scenario.get('boundary') != None:
            boundary = scenario.get('boundary')
            import matplotlib.pyplot as plt
            xlim, ylim = [boundary.x_min, boundary.x_max], [boundary.y_min, boundary.y_max]
            if config.invert: xlim = [boundary.x_max, boundary.x_min]
            plt.xlim(xlim)
            plt.ylim(ylim)
        self.clock = cu.system.Clock(scenario['frequency'] *config.speed)
        print(scenario['frequency'])
        self.clock_viz = cu.system.Clock(100)

        self.agents = []

        self.num_replay = config.num_replay
        self.thread_replay = threading.Thread(target=self.replay)
        self.thread_replay.start()
        return
    
    def replay(self):
        for _ in tqdm(range(self.num_replay)):
            self.replay_once(self.index)
        print('[ReplayRecord] finish replay')
        return

    def replay_once(self, index):
        agent_keys = [key for key in self._record.keys() if key.startswith('agent')]
        obstacle_keys = [key for key in self._record.keys() if key.startswith('obstacle')]
        timestamps = set()
        for agent_key in agent_keys:
            timestamps.update( set(self._record[agent_key].keys()) )
        timestamps = sorted(list(timestamps))

        print('[replay_once] len of timestamps: ', len(timestamps))
        for timestamp in timestamps:
            self.clock.tick_begin()

            agents = [self._record[agent_key].get(timestamp, None) for agent_key in agent_keys]
            obstacles = [self._record[obstacle_key].get(timestamp, None) for obstacle_key in obstacle_keys]
            self.agents = [i.agent for i in agents + obstacles if i != None]

            self.clock.tick_end()
        return


    def run(self):
        while True:
            self.clock_viz.tick_begin()
            self.vv.run_step(self.agents, [])
            self.vv.update_vis()
            self.clock_viz.tick_end()



class ReplayRecords(ReplayRecord):
    """
        Only for one scenario.
    """
    def __init__(self, config):
        indices = list(range(*config.indices))
        self.record_file_paths = []
        self.indices = []
        for index in indices:
            file_path = join(config.dir, str(index) + '.txt')
            if os.path.isfile(file_path):
                self.record_file_paths.append(file_path)
                self.indices.append(index)
        
        record_init = cu.rl_template.Recorder.load_from_disk(self.record_file_paths[0])
        scenario = record_init.pop('scenario')

        map_path = join(config.dir, scenario['map_name'] + '.txt')
        town_map = cu.rl_template.Recorder.load_from_disk(map_path)
        config.set('town_map', town_map)

        self.vv = config.viz_type(config)
        # if scenario.get('boundary') != None:
        #     boundary = scenario.get('boundary')
        #     import matplotlib.pyplot as plt
        #     plt.xlim([boundary.x_min, boundary.x_max])
        #     plt.ylim([boundary.y_min, boundary.y_max])
        self.clock = cu.system.Clock(scenario['frequency'] *config.speed)
        print(cu.basic.prefix(self) + 'scenario frequency: ', scenario['frequency'])
        print('\n')
        self.clock_viz = cu.system.Clock(100)

        self.agents = []

        self.num_replay = config.num_replay
        self.thread_replay = threading.Thread(target=self.replay)
        self.thread_replay.start()
        return

    def replay(self):
        for index, file_path in zip(self.indices, self.record_file_paths):
            print('\n\n' + cu.basic.prefix(self) + 'episode index: ', index)
            record = cu.rl_template.Recorder.load_from_disk(file_path)
            self._record = record
            for _ in tqdm(range(self.num_replay)):
                self.replay_once(index)
        return



def generate_args():
    from carla_utils.utils import default_argparser
    argparser = default_argparser()
    argparser.add_argument('--invert', action='store_true', help='invert axis (default: False)')

    argparser.add_argument('-d', dest='description', default='Nothing', help='[Method] description.')

    argparser.add_argument('-n', dest='num_replay', default=1, type=int, help='num_replay.')

    argparser.add_argument('-s', '--speed', dest='speed', default=1.0, type=float, help='play speed.')

    argparser.add_argument('--dir', default='./', type=str, help='')
    argparser.add_argument('--index', default=-1, type=int, help='')
    argparser.add_argument('--indices', default=[-1,100], type=int, nargs='+', help='')
    argparser.add_argument('-t', '--type', type=int, choices=[0, 1], default=0, help='0: open3d, 1: plt')

    args = argparser.parse_args()
    return args



if __name__ == "__main__":
    config = YamlConfig()
    args = generate_args()
    config.update(args)

    ### mode 1
    if config.index > 0:
        RR = ReplayRecord
    elif config.indices[0] > 0:
        RR = ReplayRecords
    else:
        raise NotImplementedError

    ### mode 2
    if config.type == 0:
        config.set('viz_type', Vopen3d)
    elif config.type == 1:
        config.set('viz_type', Vplt)
    else:
        raise NotImplementedError

    rr = RR(config)
    rr.run()


