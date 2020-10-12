#!/usr/bin/env python

# Python
import numpy as np

# ROS
import rospy
import tf

# ROS msgs
from voxgraph_msgs.msg import LoopClosure
from visualization_msgs.msg import Marker, MarkerArray


class PlaceRecognitionSimulator(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.update_period = rospy.get_param('~update_period', 1.0)
        self.world_frame = rospy.get_param('~world_frame', )
        self.ground_truth_pose_tf_frame_id = rospy.get_param(
            '~ground_truth_pose_tf_frame_id', )
        self.recognizeable_areas = rospy.get_param('~recognizeable_areas', )

        # Initialize variables
        for recognizeable_area in self.recognizeable_areas:
            recognizeable_area.observations = {}

        # Setup ROS interfaces
        self.update_timer = rospy.Timer(rospy.Duration(self.update_period),
                                        self.place_recognition_callback)
        self.loop_closure_pub = rospy.Publisher('loop_closures', LoopClosure)
        self.loop_closure_vis_pub = rospy.Publisher('loop_closure_vis', Marker)
        self.recognizeable_area_vis_pub = rospy.Publisher(
            'recognizeable_areas', MarkerArray)
        self.tf_listener = tf.TransformListener()

    def place_recognition_callback(self, _):
        # Get the current ground truth pose
        current_time = rospy.Time(0)
        (current_position,
         current_rotation) = self.tf_listener.lookupTransform(
             self.ground_truth_pose_tf_frame_id, self.world_frame,
             current_time)

        # Simulate place recognition for recognizeable areas
        self.visualize_recognizeable_areas(self.recognizeable_areas)
        for recognizeable_area in self.recognizeable_areas:
            current_distance = np.linalg.norm(
                recognizeable_area.center_position - current_position)
            if current_distance <= recognizeable_area.detection_radius:
                # Test against all previous visits
                for (previous_visit_time, previous_visit_transform) in \
                        recognizeable_area.previous_visits:
                    # TODO(victorr): Sample
                    uniform_sample = 0.2
                    if uniform_sample < recognizeable_area.detection_probability:
                        loop_closure_msg = LoopClosure()
                        loop_closure_msg.from_timestamp = previous_visit_time
                        loop_closure_msg.to_timestamp = current_time

                        loop_closure_msg.transform.translation =\
                            previous_visit_transform.position - current_position
                        loop_closure_msg.transform.rotation =\
                            previous_visit_transform.rotation - current_rotation

                        self.visualize_loop_closure(loop_closure_msg)
                        self.loop_closure_pub.publish(loop_closure_msg)

                # Log the current visit
                recognizeable_area.previous_visits[current_time] = {
                    "position": current_position,
                    "rotation": current_rotation
                }

    def visualize_loop_closure(self, loop_closure_msg):
        pass
        self.loop_closure_vis_pub

    def visualize_recognizeable_areas(self, recognizeable_areas):
        for recognizeable_area in recognizeable_areas:
            pass
        self.recognizeable_area_vis_pub


if __name__ == '__main__':
    rospy.init_node('place_recognition_simulator', anonymous=False)
    place_recognition_simulator = PlaceRecognitionSimulator()
    rospy.spin()
