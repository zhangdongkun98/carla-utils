import rldev
import carla_utils as cu
from carla_utils import carla

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import copy

from ..system import Clock
from ..world_map import Core
from ..augment import ActorVertices




class Visualizer(object):
    def __init__(self, config):
        self.config = config
        self.clock = Clock(100)

        self.vis_index = config.get('vis_index', 0)
        self.window_name = config.get('window_name', 'Vehicles Visualisation Example')
        self.title_name = config.host + ':' + str(config.port)
        self.fig = config.get('fig', None)
        if self.fig == None:
            self.fig = plt.figure()
        plt.subplots_adjust(
            left=0.05, right=0.9,
            top=0.95, bottom=0.05,
            wspace=0.4, hspace=0.4,
        )
        self.fig.canvas.set_window_title(self.window_name)
        self.num_rows = config.get('num_rows', 1)
        self.num_columns = config.get('num_columns', 1)

        self.connect_to_server()

        self.create_axis()
        if config.invert:
            self.ax.invert_xaxis()
        self.update_vis()

        self.run_once()

        self.foreground = []
        return


    def connect_to_server(self):
        self.config.set('mode', None)
        self.core = Core(self.config, use_tm=False)
        self.world = self.core.world
        self.town_map = self.core.town_map


    def get_scenario_boundary(self):
        waypoints = self.town_map.generate_waypoints(2)
        margin = 10
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin
        boundary = rldev.Data(x_min=min_x, y_min=min_y, x_max=max_x, y_max=max_y)
        return self.config.get('boundary', boundary)


    def create_axis(self):
        boundary = self.get_scenario_boundary()

        self.ax = self.fig.add_subplot(self.num_rows, self.num_columns, self.vis_index+1)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_facecolor(np.array([180, 180, 180], np.float64) / 255)

        self.ax.set_title(self.title_name)
        self.ax.set_xlim([boundary.x_min, boundary.x_max])
        self.ax.set_ylim([boundary.y_min, boundary.y_max])



    def update_vis(self):
        plt.pause(0.00001)
        return


    def run_once(self):
        xlim, ylim = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.clear()
        self.ax.set_title(self.title_name)
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        if not hasattr(self, 'background_topology'):
            topology = cu.get_topology(self.town_map, sampling_resolution=2.0)
            self.background_topology = [t.info for t in topology]
            self.junctions = cu.get_junctions(topology)
        for t in self.background_topology:
            line = self.ax.plot(t.x, t.y)[0]
        for junction in self.junctions:
            vertices = ActorVertices.bbx(junction.bounding_box)
            vertices = np.vstack([vertices, vertices[[0]]])
            self.ax.plot(vertices[:,0], vertices[:,1], '-', linewidth=3)
        
        self.update_vis()
        return



    def run_step(self, vehicles, walkers):
        if not hasattr(self, 'timestamp'):
            self.timestamp = time.time()
        if time.time() - self.timestamp > 10:
            self.timestamp = time.time()
            self.run_once()

        [i.remove() for i in self.foreground]
        self.foreground = []
        self.ax.patches = []

        for actor in vehicles + walkers:
            if actor.type_id.startswith('walker'):
                color = '0,0,0'
            else:
                color = actor.attributes.get('color', '190,190,190')
            color = np.array(eval(color)).astype(np.float64) / 255

            if cu.Role.loads(actor.attributes['role_name']).atype == cu.ScenarioRole.learnable:
                origin = ActorVertices.origin(actor)
                r = patches.Rectangle(origin.point, origin.y_length, origin.x_length, angle=np.rad2deg(origin.theta)-90, color=color, alpha=0.75)
                self.ax.add_patch(r)
            
            vertices, lines = ActorVertices.d2arrow(actor)
            vertices = np.vstack([vertices, vertices[[4]], vertices[[0]]])

            line = self.ax.plot(vertices[:,0], vertices[:,1], '-', color=color)[0]
            self.foreground.append(line)
        return

    
    def run(self):
        while True:
            self.clock.tick_begin()

            t1 = time.time()
            actors = self.world.get_actors()
            vehicles = actors.filter('*vehicle*')
            walkers = actors.filter('walker*')
            self.run_step(list(vehicles), list(walkers))
            t2 = time.time()
            self.update_vis()
            t3 = time.time()

            res = 'freq: '
            for t in [t3-t1, t2-t1, t3-t2]:
                res += '{:0=6.2f}, '.format(1/t)
            res = res[:-2]
            print(res, end='\r', flush=True)

            self.clock.tick_end()




class BatchVisualizer(object):
    visualize_cls = Visualizer

    def __init__(self, config: rldev.YamlConfig):
        self.config = config
        self.clock = Clock(100)

        num_envs = len(config.ports)
        num_rows = config.num_rows
        assert num_envs % num_rows == 0
        num_columns = num_envs // num_rows

        visualize_cls = config.get('visualize_cls', self.visualize_cls)

        self.window_name = 'Vehicles Visualisation Example'
        fig = plt.figure()
        fig.canvas.set_window_title(self.window_name)
        config.set('window_name', self.window_name)
        config.set('fig', fig)
        config.set('num_rows', num_rows)
        config.set('num_columns', num_columns)

        self.env_keys = [(config.host, p) for p in config.ports]
        self.env_visualizers = []
        for i, port in enumerate(config.ports):
            cfg = copy.copy(config)
            cfg.port = port
            cfg.set('vis_index', i)
            env_visualizer = visualize_cls(cfg)
            self.env_visualizers.append(env_visualizer)

        self.update_vis()
        return


    def update_vis(self):
        plt.pause(0.00001)
        return


    def run(self):
        while True:
            self.clock.tick_begin()

            for env_visualizer in self.env_visualizers:
                actors = env_visualizer.world.get_actors()
                vehicles = actors.filter('*vehicle*')
                walkers = actors.filter('walker*')
                env_visualizer.run_step(list(vehicles), list(walkers))
            self.update_vis()

            self.clock.tick_end()






def generate_args():
    from carla_utils.utils import default_argparser
    argparser = default_argparser()

    argparser.add_argument('--batch', action='store_true', help='batch viz env (default: False)')
    argparser.add_argument('--invert', action='store_true', help='invert axis (default: False)')
    argparser.add_argument('--ports', default=[2000,2002,2004,2006], type=int, nargs='+', help='')
    argparser.add_argument('--num-rows', default=1, type=int, help='batch viz, number of rows.')

    args = argparser.parse_args()
    return args


if __name__ == "__main__":
    config = cu.basic.YamlConfig()
    args = generate_args()
    config.update(args)

    if not config.batch:
        VV = Visualizer
    else:
        VV = BatchVisualizer
    visualizer = VV(config, )
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print('\ncanceled by user')


