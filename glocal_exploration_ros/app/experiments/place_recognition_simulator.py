#!/usr/bin/env python

# Python
import numpy as np

# ROS
import rospy
import tf

# ROS msgs
from voxgraph_msgs.msg import LoopClosure
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PlaceRecognitionSimulator(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.update_period = rospy.get_param('~update_period', 1.0)
        self.world_frame = rospy.get_param('~world_frame', 'odom')
        self.ground_truth_pose_tf_frame_id = rospy.get_param(
            '~ground_truth_pose_tf_frame_id', 'airsim_drone/Lidar_ground_truth')
        self.recognizeable_areas = rospy.get_param('~recognizeable_areas')
        self.loop_closure_topic = rospy.get_param(
            '~loop_closure_output_topic',
            '/glocal/glocal_system/loop_closure_input')
        self.submapping_interval = rospy.get_param('~submapping_interval_s', 10.0)

        # Initialize variables
        for recognizeable_area in self.recognizeable_areas:
            recognizeable_area['previous_visits'] = []
        self.loop_closure_publication_queue = []
        self.submapping_interval = rospy.Duration(self.submapping_interval)

        # Setup ROS interfaces
        self.update_timer = rospy.Timer(rospy.Duration(self.update_period),
                                        self.place_recognition_callback)
        self.loop_closure_pub = rospy.Publisher(self.loop_closure_topic, LoopClosure, queue_size=100)
        self.loop_closure_vis_pub = rospy.Publisher('loop_closure_vis', Marker, queue_size=100)
        self.recognizeable_area_vis_pub = rospy.Publisher(
            'recognizeable_areas', MarkerArray, queue_size=100)
        self.tf_listener = tf.TransformListener()


    def place_recognition_callback(self, _):
        # Publish enqueued messages once they are old enough
        current_time = rospy.Time.now()
        for (idx, place_recognition_msg) in enumerate(self.loop_closure_publication_queue):
            if place_recognition_msg.to_timestamp + rospy.Duration(12) < current_time:
                rospy.loginfo('Publishing loop closure from timestamp %s to %s'
                              % (place_recognition_msg.from_timestamp,
                                 place_recognition_msg.to_timestamp))
                self.loop_closure_pub.publish(place_recognition_msg)
                self.loop_closure_publication_queue.pop(idx)

        # Get the current ground truth pose
        current_position = None
        current_pose = None
        try:
            (current_position,
             current_rotation) = self.tf_listener.lookupTransform(
                 self.world_frame, self.ground_truth_pose_tf_frame_id,
                 rospy.Time(0))
            current_pose = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(current_position),
                tf.transformations.quaternion_matrix(current_rotation))

        except tf.Exception:
            rospy.logwarn('Could not (yet) lookup transform from %s to %s in '
                          'place recognition simulator.' % (
                self.ground_truth_pose_tf_frame_id, self.world_frame))
            return

        # Simulate place recognition for recognizeable areas
        self.visualize_recognizeable_areas(self.recognizeable_areas)
        for recognizeable_area in self.recognizeable_areas:
            area_center_position =\
                np.array(recognizeable_area['center_position'])
            current_distance = np.linalg.norm(
                area_center_position - current_position)
            if current_distance <= recognizeable_area['detection_radius']:
                # Test against all previous visits
                for previous_visit in \
                        recognizeable_area['previous_visits']:
                    if self.submapping_interval < current_time - previous_visit['time']:
                        uniform_sample = np.random.uniform(low=0.0, high=1.0)
                        if uniform_sample < recognizeable_area['detection_probability']:
                            loop_closure_msg = LoopClosure()
                            loop_closure_msg.from_timestamp = previous_visit['time']
                            loop_closure_msg.to_timestamp = current_time

                            T_previous_current = \
                                tf.transformations.concatenate_matrices(
                                    tf.transformations.inverse_matrix(previous_visit['pose']), current_pose)
                            t_previous_current = \
                                tf.transformations.translation_from_matrix(T_previous_current)
                            r_previous_current = \
                                tf.transformations.quaternion_from_matrix(T_previous_current)

                            # Simulate imperfect relative poses
                            stddev_xy = 0.15
                            stddev_z = 0.05
                            stddev_yaw = 0.05
                            t_previous_current[0] += np.random.normal(0.0, stddev_xy)
                            t_previous_current[1] += np.random.normal(0.0, stddev_xy)
                            t_previous_current[2] += np.random.normal(0.0, stddev_z)

                            yaw_noise = np.random.normal(0.0, stddev_yaw)
                            yaw_noise_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_noise)
                            r_previous_current = tf.transformations.quaternion_multiply(r_previous_current, yaw_noise_quat)

                            loop_closure_msg.transform.translation.x = t_previous_current[0]
                            loop_closure_msg.transform.translation.y = t_previous_current[1]
                            loop_closure_msg.transform.translation.z = t_previous_current[2]
                            loop_closure_msg.transform.rotation.x = r_previous_current[0]
                            loop_closure_msg.transform.rotation.y = r_previous_current[1]
                            loop_closure_msg.transform.rotation.z = r_previous_current[2]
                            loop_closure_msg.transform.rotation.w = r_previous_current[3]

                            self.loop_closure_publication_queue.append(loop_closure_msg)

                            start_point = tf.transformations.translation_from_matrix(previous_visit['pose'])
                            end_point = tf.transformations.translation_from_matrix(current_pose)
                            self.visualize_loop_closure(start_point, end_point)

                # Log the current visit
                recognizeable_area['previous_visits'].append({
                    "time": current_time,
                    "pose": current_pose
                })

    def visualize_loop_closure(self, start_point, end_point):
        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time(0)
        marker.ns = "place_recognition_association"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        start_point_msg = Point()
        start_point_msg.x = start_point[0]
        start_point_msg.y = start_point[1]
        start_point_msg.z = start_point[2]
        marker.points.append(start_point_msg)
        end_point_msg = Point()
        end_point_msg.x = end_point[0]
        end_point_msg.y = end_point[1]
        end_point_msg.z = end_point[2]
        marker.points.append(end_point_msg)

        self.loop_closure_vis_pub.publish(marker)

    def visualize_recognizeable_areas(self, recognizeable_areas):
        marker_array = MarkerArray()
        for idx, recognizeable_area in enumerate(recognizeable_areas):
            marker = Marker()

            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "recognizeable_areas"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = recognizeable_area['center_position'][0]
            marker.pose.position.y = recognizeable_area['center_position'][1]
            marker.pose.position.z = recognizeable_area['center_position'][2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * recognizeable_area['detection_radius']
            marker.scale.y = 2 * recognizeable_area['detection_radius']
            marker.scale.z = 2 * recognizeable_area['detection_radius']
            marker.color.a = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.recognizeable_area_vis_pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('place_recognition_simulator', anonymous=False)
    place_recognition_simulator = PlaceRecognitionSimulator()
    rospy.spin()
