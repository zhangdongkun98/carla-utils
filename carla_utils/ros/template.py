import rospy

import time
import subprocess

from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


from ..world_map import connect_to_server
from .pub_sub import ROSPublish, PubFormat, basic_publish
from .create_message import Map, VehiclesTransform, BoundingBoxes



pub_dict = dict()

########### /tf_static ###########
def update_tf_static(publisher, args):
    sensor_manager, vehicle_frame_id = args[0], args[1]
    tfmsg = TFMessage()
    for sensor_master in sensor_manager.sensor_list:
        tf_stamped = ru.get_sensor_tf_stamped(sensor_master, vehicle_frame_id)
        tfmsg.transforms.append(tf_stamped)
    publisher.publish(tfmsg)



pub_dict['~town_map'] = PubFormat(MarkerArray, basic_publish, True, 1)
pub_dict['/tf'] = PubFormat(TFMessage, basic_publish, False, 1)
pub_dict['~vehicle_bbx'] = PubFormat(MarkerArray, basic_publish, False, 1)
# pub_dict['/tf_static'] = PubFormat(TFMessage, update_tf_static, True, 1)





class RosViz(object):
    def __init__(self, config):
        host, port, timeout, map_name = config.host, config.port, config.timeout, config.map_name
        self.client, self.world, self.town_map = connect_to_server(host, port, timeout)

        rospy.init_node('carla')
        self.clock = rospy.Rate(10)
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
        self.ros_pubish.publish(topic, Map(self.global_frame_id, timestamp, self.town_map))



    def publish(self, timestamp):
        actors = self.world.get_actors()
        vehicles = actors.filter('*vehicle*')

        # for i in self.world.get_environment_objects(): print(i)
        # print()
        
        topic = '~town_map'
        self.ros_pubish.publish(topic, Map(self.global_frame_id, timestamp, self.town_map))

        topic = '/tf'
        self.ros_pubish.publish(topic, VehiclesTransform(self.global_frame_id, timestamp, vehicles))

        topic = '~vehicle_bbx'
        self.ros_pubish.publish(topic, BoundingBoxes(None, timestamp, vehicles))
        return


    def run(self):
        self.publishOnce()
        while not rospy.is_shutdown():
            self.publish(time.time())
            self.clock.sleep()


def start_rviz():
    try:
        rospy.get_master().getSystemState()
    except:
        subprocess.Popen('roscore')
        time.sleep(1)
    subprocess.Popen('rosrun rviz rviz -d utils_other/carla.rviz', shell=True)
    return


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
        config = parse_yaml_file_unsafe(join(file_dir, './default_carla.yaml'))
    args = generate_args()
    config.update(args)
    
    ros_viz = RosViz(config, )
    try:
        ros_viz.run()
    except KeyboardInterrupt:
        print('canceled by user')
