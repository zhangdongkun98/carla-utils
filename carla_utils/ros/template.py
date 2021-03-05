import carla
import rospy

import time
import subprocess
import multiprocessing as mp

from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from ..system import Clock
from ..world_map import connect_to_server
from .pub_sub import ROSPublish, PubFormat, basic_publish
from . import create_message as cm


pub_dict = dict()


pub_dict['~town_map'] = PubFormat(MarkerArray, basic_publish, True, 1)
pub_dict['/tf'] = PubFormat(TFMessage, basic_publish, False, 1)
pub_dict['~vehicle_bbx'] = PubFormat(MarkerArray, basic_publish, False, 1)
# pub_dict['/tf_static'] = PubFormat(TFMessage, update_tf_static, True, 1)





class RosViz(object):
    def __init__(self, config):
        host, port, timeout, map_name = config.host, config.port, config.timeout, config.map_name
        self.client, self.world, self.town_map = connect_to_server(host, port, timeout)

        rospy.init_node('carla')
        self.ros_clock = rospy.Rate(50)
        self.global_frame_id = 'map'
        self.ros_pubish = ROSPublish(pub_dict,
            '/tf',
            # '/tf', '/tf_static',
            '~vehicle_bbx',
            '~town_map',
        )


    def publishOnce(self):
        timestamp = time.time()
        # topic = '/tf_static'
        # self.ros_pubish.publish(topic, (self.sensor_manager, self.vehicle_frame_id))

        topic = '~town_map'
        self.ros_pubish.publish(topic, cm.Map(self.global_frame_id, timestamp, self.town_map))
        return


    def publish(self, timestamp):
        actors = self.world.get_actors()
        vehicles = actors.filter('*vehicle*')
        static_vehicles = None
        if hasattr(self.world, 'get_environment_objects'):
            static_vehicles = self.world.get_environment_objects(carla.CityObjectLabel.Vehicles)

        topic = '/tf'
        tfmsg = cm.VehiclesTransform(self.global_frame_id, timestamp, vehicles)
        if static_vehicles != None:
            tfmsg.transforms.extend(cm.StaticVehiclesTransform(self.global_frame_id, timestamp, static_vehicles).transforms)
        self.ros_pubish.publish(topic, tfmsg)

        topic = '~vehicle_bbx'
        bbx = cm.BoundingBoxes(None, timestamp, vehicles)
        if static_vehicles != None:
            bbx.markers.extend(cm.StaticBoundingBoxes(None, timestamp, static_vehicles).markers)
        self.ros_pubish.publish(topic, bbx)
        return


    def run(self):
        self.publishOnce()
        while not rospy.is_shutdown():
            t1 = time.time()

            self.publish(time.time())

            t2 = time.time()
            # print('time: ', t2-t1, 1/(t2-t1))

            self.ros_clock.sleep()


def start_rviz():
    try:
        rospy.get_master().getSystemState()
    except:
        subprocess.Popen('roscore')
        time.sleep(1)
    subprocess.Popen('rosrun rviz rviz -d utils_other/carla.rviz', shell=True)
    return

def start_repub():
    #     <arg name="input_topic" value="env_info"/>
    # <arg name="output_fields" value="road_path obstacle_array"/>
    # <node pkg="carla_msgs" type="RepubField" name="repub_field"
    #     args="$(arg input_topic) $(arg output_fields)"
    #     ns="ego_vehicle" output="screen">
    pass



if __name__ == "__main__":
    start_rviz()
    time.sleep(0.5)
    
    import os
    from os.path import join
    from ..system import parse_yaml_file_unsafe
    from ..utils.tools import generate_args

    try:
        config = parse_yaml_file_unsafe('./config/carla.yaml')
    except FileNotFoundError:
        print('[vehicle_visualizer] use default config.')
        file_dir = os.path.dirname(__file__)
        config = parse_yaml_file_unsafe(join(file_dir, '../utils/default_carla.yaml'))
    args = generate_args()
    config.update(args)
    
    ros_viz = RosViz(config, )
    try:
        ros_viz.run()
    except KeyboardInterrupt:
        print('canceled by user')
