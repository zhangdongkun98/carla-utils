
import rospy, tf

import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



def header(frame_id, timestamp):
    hd = Header()
    hd.frame_id = frame_id
    hd.stamp = rospy.Time.from_sec(timestamp)
    return hd


def CarlaWaypointToGeoPoint(waypoint):
    point = Point()
    point.x = waypoint.transform.location.x
    point.y = waypoint.transform.location.y
    point.z = waypoint.transform.location.z
    return point



'''
void convertLocationInPlace(carla::geom::Location& in) {
  in.y = -in.y;
  return;
}


void convertRotationInPlace(carla::geom::Rotation& in) {
  in.roll = -in.roll;
  in.yaw = -in.yaw;
  return;
}
'''

def CarlaTransformToGeoTransformStamped(frame_id, timestamp, child_frame_id, transform):
    geo_transform = TransformStamped()
    geo_transform.header = header(frame_id, timestamp)
    geo_transform.child_frame_id = child_frame_id
    geo_transform.transform.translation.x = transform.location.x
    geo_transform.transform.translation.y = transform.location.y
    geo_transform.transform.translation.z = transform.location.z

    roll, pitch, yaw = transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw
    
    quaternion = tf.transformations.quaternion_from_euler(
            np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
    geo_transform.transform.rotation.x = quaternion[0]
    geo_transform.transform.rotation.y = quaternion[1]
    geo_transform.transform.rotation.z = quaternion[2]
    geo_transform.transform.rotation.w = quaternion[3]
    return geo_transform


class NonCoConvertion(object):
    @staticmethod
    def PesudoHeaderToHeader(frame_id, time_stamp):
        header = Header()
        header.frame_id = frame_id
        header.stamp = rospy.Time.from_sec(time_stamp)
        return header


    @staticmethod
    def CarlaStateToTransformStamped(state, vehicle_frame_id):
        tf_stamped = TransformStamped()
        tf_stamped.header = NonCoConvertion.PesudoHeaderToHeader(state.frame_id, state.time_stamp)
        tf_stamped.child_frame_id = vehicle_frame_id
        tf_stamped.transform.translation.x = state.x
        tf_stamped.transform.translation.y = state.y
        tf_stamped.transform.translation.z = state.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, state.theta)
        tf_stamped.transform.rotation.x = quaternion[0]
        tf_stamped.transform.rotation.y = quaternion[1]
        tf_stamped.transform.rotation.z = quaternion[2]
        tf_stamped.transform.rotation.w = quaternion[3]
        return tf_stamped

    @staticmethod
    def CarlaWaypointToPoseStamped(frame_id, time_stamp, waypoint):
        '''
            waypoint: carla.Waypoint
            frame_id: global frame_id
        '''
        pose_stamped = PoseStamped()
        pose_stamped.header = NonCoConvertion.PesudoHeaderToHeader(frame_id, time_stamp)
        pose_stamped.pose.position.x = waypoint.transform.location.x
        pose_stamped.pose.position.y = waypoint.transform.location.y
        pose_stamped.pose.position.z = waypoint.transform.location.z

        roll_rad = np.deg2rad(waypoint.transform.rotation.roll)
        pitch_rad= np.deg2rad(waypoint.transform.rotation.pitch)
        yaw_rad  = np.deg2rad(waypoint.transform.rotation.yaw)
        quaternion = tf.transformations.quaternion_from_euler(
          roll_rad, pitch_rad, yaw_rad)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        return pose_stamped
    @staticmethod
    def CUAWaypointToPoseStamped(waypoint):
        '''
            waypoint: carla_uitls/augment/Waypoint
        '''
        pose_stamped = PoseStamped()
        pose_stamped.header = NonCoConvertion.PesudoHeaderToHeader(waypoint.frame_id, waypoint.time_stamp)
        pose_stamped.pose.position.x = waypoint.x
        pose_stamped.pose.position.y = waypoint.y
        pose_stamped.pose.position.z = waypoint.z

        quaternion = tf.transformations.quaternion_from_euler(
            waypoint.roll_rad, waypoint.pitch_rad, waypoint.yaw_rad)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        return pose_stamped

    @staticmethod
    def CarlaWaypointsToNavPath(frame_id, time_stamp, waypoints):
        '''
             waypoints: list of carla.Waypoint
        '''
        path = Path()
        path.header = NonCoConvertion.PesudoHeaderToHeader(frame_id, time_stamp)
        path.poses = [NonCoConvertion.CarlaWaypointToPoseStamped(frame_id, time_stamp, waypoint) for waypoint in waypoints]
        return path
    @staticmethod
    def CUAWaypointsToNavPath(frame_id, time_stamp, waypoints):
        '''
            waypoint: carla_uitls/augment/Waypoint
        '''
        path = Path()
        path.header = NonCoConvertion.PesudoHeaderToHeader(frame_id, time_stamp)
        path.poses = [NonCoConvertion.CUAWaypointToPoseStamped(waypoint) for waypoint in waypoints]
        return path

    @staticmethod
    def CUAGlobalPathToNavPath(global_path):
        '''
            global_path: carla_uitls/augment/GlobalPath
        '''
        path = Path()
        frame_id, time_stamp = global_path.frame_id, global_path.time_stamp
        path.header = NonCoConvertion.PesudoHeaderToHeader(frame_id, time_stamp)
        path.poses = [NonCoConvertion.CarlaWaypointToPoseStamped(frame_id, time_stamp, waypoint) for waypoint in global_path.carla_waypoints]
        return path