import rldev
import carla_utils as cu
from carla_utils import carla
import rospy
import rostopic

import copy
import time
import subprocess
import os
import signal
import multiprocessing as mp

import genpy
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from ..world_map import Core

from carla_utils.ros import PublishWrapper
from carla_utils.ros.pub_sub import PubFormat, basic_publish
from carla_utils.ros import create_message as cm
from carla_utils.ros import convert as cvt



class RosViz(PublishWrapper):
    pub_dict = {
        '~town_map': PubFormat(MarkerArray, basic_publish, True, 1),
        '/tf': PubFormat(TFMessage, basic_publish, False, 1),
        '~vehicle_bbx': PubFormat(MarkerArray, basic_publish, False, 1),
    }

    def __init__(self, config: cu.basic.YamlConfig):
        node_name = 'carla'
        core = Core(config, use_tm=False)
        self.client, self.world, self.town_map = core.client, core.world, core.town_map
        self.town_map_name = self.town_map.name

        super().__init__(config, node_name)

        self.ros_clock = rospy.Rate(100)



    def run_once(self):
        timestamp = time.time()
        # topic = '/tf_static'
        # self.ros_pubish.publish(topic, (self.sensor_manager, self.vehicle_frame_id))

        topic = '~town_map'
        self.map_viz = cm.Map(self.global_frame_id, time.time(), self.town_map)
        self.map_viz.markers[0].header = cvt.header(self.global_frame_id, timestamp)
        self.map_viz.markers[1].header = cvt.header(self.global_frame_id, timestamp)
        self.ros_pubish.publish(topic, self.map_viz)

        if self.world.get_map().name != self.town_map_name:
            self.world = self.client.get_world()
            self.town_map = self.world.get_map()
            self.town_map_name = self.town_map.name
        return


    def run_step(self):
        timestamp = time.time()
        actors = self.world.get_actors()
        vehicles = list(actors.filter('*vehicle*'))
        sensors = list(actors.filter('*sensor*'))
        static_vehicles = None
        if hasattr(self.world, 'get_environment_objects'):
            static_vehicles = self.world.get_environment_objects(carla.CityObjectLabel.Vehicles)

        topic = '/tf'
        tfmsg = cm.VehiclesTransform(self.global_frame_id, timestamp, vehicles)
        tfmsg.transforms.extend(cm.SensorsTransform(self.global_frame_id, timestamp, sensors).transforms)
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
        self.run_once()
        while not rospy.is_shutdown():
            t1 = time.time()

            self.run_step()
            t2 = time.time()
            # print('time: ', t2-t1, 1/(t2-t1))

            self.ros_clock.sleep()




def reset_header_timestamp_slow(msg, timestamp):
    attrs = [attr for attr in dir(msg) if not attr.startswith('_')]
    attrs = [attr for attr in attrs if not callable(getattr(msg, attr))]

    for attr_name in attrs:
        attr = getattr(msg, attr_name)
        if isinstance(attr, list):
            for a in attr:
                reset_header_timestamp_slow(a, timestamp)
        elif isinstance(attr, Header):
            msg.header.stamp = timestamp
        elif isinstance(msg, genpy.Message):
            reset_header_timestamp_slow(attr, timestamp)
        else:
            pass
    return



def reset_header_timestamp(msg, timestamp):
    attrs = [attr for attr in dir(msg) if not attr.startswith('_')]
    attrs = [attr for attr in attrs if not callable(getattr(msg, attr))]

    for attr_name in attrs:
        attr = getattr(msg, attr_name)
        if isinstance(attr, list):
            for a in attr:
                if hasattr(a, 'header'):
                    a.header.stamp = timestamp
        elif isinstance(attr, Header):
            msg.header.stamp = timestamp
        else:
            pass
    return




class RepubTopic(object):
    def __init__(self, repub_topic, repub_frequency=10.0):
        self.repub_topic = repub_topic
        topic_class, topic_name, _ = rostopic.get_topic_class(repub_topic)

        print(rldev.prefix(self) + 'repub topic_name: ', topic_name)

        self.msg = None
        self.publisher = rospy.Publisher(topic_name + '_repub', topic_class, latch=False, queue_size=1)
        self.subscriber = rospy.Subscriber(topic_name, topic_class, callback=self.callback)
        self.clock = rospy.Rate(repub_frequency)
        return


    def callback(self, msg):
        self.msg = msg


    def run_step(self):
        if self.msg != None:
            reset_header_timestamp(self.msg, rospy.Time.from_sec(time.time()))
            self.publisher.publish(self.msg)
        return


    def run(self):
        while not rospy.is_shutdown():
            t1 = time.time()

            self.run_step()
            t2 = time.time()
            # print('time: ', t2-t1, 1/(t2-t1))

            self.clock.sleep()
        return




def start_rviz(config):
    try:
        rospy.get_master().getSystemState()
    except:
        subprocess.Popen('roscore')
        time.sleep(1)
    
    cmd_str = 'rosrun rviz rviz -d {}/carla_{}_{}.rviz'.format(config.dir_rviz, config.host.replace('.', '_'), str(config.port))
    print('run cmd:\n    ', cmd_str)
    rviz = subprocess.Popen(cmd_str, shell=True)
    return rviz


def start_repub(repub_topics):
    from threading import Thread as MultiTask
    # from multiprocessing import Process as MultiTask

    rpts = [RepubTopic(repub_topic) for repub_topic in repub_topics]
    processes = [MultiTask(target=rpt.run) for rpt in rpts]
    [p.start() for p in processes]
    return processes




if __name__ == "__main__":
    config = cu.basic.YamlConfig()
    from carla_utils.utils import default_argparser
    argparser = default_argparser()
    argparser.add_argument('--dir-rviz', default='./', type=str, help='dir of .rviz file.')
    argparser.add_argument('--repub-topics', default=[], type=str, nargs='+', help='')
    args = argparser.parse_args()
    config.update(args)

    rviz = start_rviz(config)
    time.sleep(0.5)

    try:
        ros_viz = RosViz(config)

        repub_processes = start_repub(config.repub_topics)

        ros_viz.run()
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        ros_viz.kill()
        rviz.send_signal(signal.SIGKILL)
        os.killpg(os.getpgid(rviz.pid), signal.SIGKILL)

