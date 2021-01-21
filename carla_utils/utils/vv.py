'''
    vehicle_visualizer
'''
print(__doc__)

import carla

import numpy as np
import open3d

from ..system import Clock, parse_yaml_file_unsafe
from ..basic import RotationMatrix, HomogeneousMatrix

from ..augment import vector3DToArray
from ..world_map import connect_to_server

from .tools import generate_args


def calculate_vis_bounding_box(vehicle : carla.Vehicle):
    current_transform = vehicle.get_transform()
    extent = vehicle.bounding_box.extent
    rotation_matrix = RotationMatrix.yaw(np.deg2rad(current_transform.rotation.yaw))

    center = vector3DToArray(current_transform.location).reshape(3,)
    center[-1] += extent.z

    color = vehicle.attributes.get('color', '190,190,190')
    color = np.array(eval(color)).astype(np.float64) / 255

    bounding_box = open3d.geometry.OrientedBoundingBox()
    bounding_box.center = center
    bounding_box.color = color
    bounding_box.x_axis = rotation_matrix[:,0] * extent.x
    bounding_box.y_axis = rotation_matrix[:,1] * extent.y
    bounding_box.z_axis = rotation_matrix[:,2] * extent.z
    return bounding_box


def get_fixed_boundary(color_open3d : np.ndarray):
    max_x, max_y = 80, 45
    z = 0
    line_set = open3d.geometry.LineSet()
    points = np.array([ [max_x,max_y,z], [-max_x,max_y,z], [-max_x,-max_y,z], [max_x,-max_y,z] ]).astype(np.float64)
    lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])

    # color_open3d = np.ones((3,), dtype=np.float32)
    color = np.expand_dims(color_open3d, axis=0).repeat(len(lines), axis=0)

    line_set.points = open3d.utility.Vector3dVector(points)
    line_set.lines = open3d.utility.Vector2iVector(lines)
    line_set.colors = open3d.utility.Vector3dVector(color)
    return line_set



class VehiclesVisualizer(object):
    def __init__(self, config, view_pose=None):
        '''parameter'''
        self.config = config
        host, port, timeout, _ = config.host, config.port, config.timeout, config.map_name
        self.client, self.world, self.town_map = connect_to_server(host, port, timeout)
        self.clock = Clock(10)
        self.max_vehicles = config.max_vehicles  ## max number

        self.window_name = "Vehicles Visualisation Example"
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(window_name=self.window_name, width=1200, height=800, left=0, top=0)
        self.view_pose = [0, 0, 60, 0, 0, -np.pi/2] if view_pose is None else view_pose

        render_option = self.vis.get_render_option()
        self.background_color = np.array([0.1529, 0.1569, 0.1333], np.float32)
        render_option.background_color = self.background_color
        render_option.point_color_option = open3d.visualization.PointColorOption.ZCoordinate
        coordinate_frame = open3d.geometry.TriangleMesh.create_coordinate_frame()
        self.vis.add_geometry(coordinate_frame)
        view_control = self.vis.get_view_control()
        params = view_control.convert_to_pinhole_camera_parameters()
        params.extrinsic = HomogeneousMatrix.xyzrpy(self.view_pose)
        view_control.convert_from_pinhole_camera_parameters(params)

        self.vis.add_geometry(get_fixed_boundary(self.background_color))

        self.bounding_boxs = [open3d.geometry.OrientedBoundingBox() for _ in range(self.max_vehicles)]
        [self.vis.add_geometry(bounding_box) for bounding_box in self.bounding_boxs]


    def run_step(self, vehicles):
        number_min = min(len(vehicles), self.max_vehicles)
        number_max = max(len(vehicles), self.max_vehicles)

        for i in range(number_min):
            vehicle, bounding_box = vehicles[i], self.bounding_boxs[i]
            new_bounding_box = calculate_vis_bounding_box(vehicle)
            bounding_box.center = new_bounding_box.center
            bounding_box.color = new_bounding_box.color
            bounding_box.x_axis = new_bounding_box.x_axis
            bounding_box.y_axis = new_bounding_box.y_axis
            bounding_box.z_axis = new_bounding_box.z_axis
        
        for i in range(number_min, number_max): self.bounding_boxs[i].clear()
        
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()

    
    def run(self):
        while True:
            self.clock.tick_begin()

            actors = self.world.get_actors()
            vehicles = actors.filter('*vehicle*')
            self.run_step(list(vehicles))

            self.clock.tick_end()




if __name__ == "__main__":
    import os
    from os.path import join

    try:
        config = parse_yaml_file_unsafe('./config/carla.yaml')
    except FileNotFoundError:
        print('[vehicle_visualizer] use default config.')
        file_dir = os.path.dirname(__file__)
        config = parse_yaml_file_unsafe(join(file_dir, './default_carla.yaml'))
    args = generate_args()
    config.update(args)
    
    vehicles_visualizer = VehiclesVisualizer(config)
    try:
        vehicles_visualizer.run()
    except KeyboardInterrupt:
        print('canceled by user')

