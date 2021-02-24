#!/usr/bin/env python

import numpy as np

import rospy
import tf
from visualization_msgs.msg import Marker


class PathVisualizer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # Params
        self.red = rospy.get_param('~r', 0)  # color 0-1
        self.green = rospy.get_param('~g', 0)
        self.blue = rospy.get_param('~b', 0)
        self.length = rospy.get_param('~length', 0.3)  # m
        self.width = rospy.get_param('~width',
                                     0.2)  # m 0.2 for vicon, 0.4 for garage
        self.distance = rospy.get_param('~distance', 0.05)  # m
        self.use_arrow = rospy.get_param('~use_arrow', False)  # m
        self.frame_name = rospy.get_param('frame_name', 'state')  # m
        self.max_count = rospy.get_param('max_count', 585)  # m
        # max_count: 585 for Vicon room, 1140 for garage

        # ROS
        self.pub = rospy.Publisher("~path", Marker, queue_size=100)
        self.tf_listener = tf.TransformListener()
        self.timer = rospy.Timer(rospy.Duration(0.001), self.pose_cb)
        self.path_sub = rospy.Subscriber('~path_in', Marker,
                                         self.path_callback)
        self.path_sub2 = rospy.Subscriber('~path_in_global', Marker,
                                          self.path2_callback)
        self.path_pub = rospy.Publisher('path_out', Marker, queue_size=10000)

        # variables
        self.previous_pose = None
        self.counter = 0
        self.path2_counter = 0
        self.prev_pos2 = None

    def pose_cb(self, _):
        # Get pose
        pose = None
        try:
            (t, _) = self.tf_listener.lookupTransform('world', self.frame_name,
                                                      rospy.Time(0))
            pose = np.array(t)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return

        # Init
        if self.previous_pose is None:
            self.previous_pose = pose
            return

        # Only plot every 'distance' meters
        dist = np.linalg.norm(self.previous_pose - pose)
        if dist < self.distance:
            return
        self.previous_pose = pose

        # Plot
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        msg.pose.orientation.w = 1
        msg.color.r = 1 - min(float(self.counter) / self.max_count, 1)
        msg.color.g = msg.color.r
        msg.color.b = msg.color.r
        msg.color.a = 1
        msg.ns = "path"
        if self.use_arrow:
            msg.type = 0
            msg.scale.x = self.length
            msg.scale.y = self.width
            msg.scale.z = self.width * 2
        else:
            # Sphere
            msg.type = 2
            msg.scale.x = self.width
            msg.scale.y = self.width
            msg.scale.z = self.width
        msg.id = self.counter
        self.counter = self.counter + 1
        self.pub.publish(msg)
        # print("Counted: %i" % self.counter)

    def path_callback(self, msg, is_local=True):
        color = [0, 0, 1]
        if is_local:
            color = [1, 0, 0]

        # Data
        if self.prev_pos2 is None:
            self.prev_pos2 = np.array(
                [msg.points[0].x, msg.points[0].y, msg.points[0].z])
        end = np.array([msg.points[-1].x, msg.points[-1].y, msg.points[-1].z])
        direction = (end - self.prev_pos2)
        n_points = int(np.linalg.norm(direction) / self.distance)
        direction = direction / np.linalg.norm(direction)

        # Plot
        for i in range(n_points):
            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "odom"
            pose = self.prev_pos2 + i * self.distance * direction
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation.w = 1
            msg.color.r = color[0]
            msg.color.g = color[1]
            msg.color.b = color[2]
            msg.color.a = 1
            msg.ns = "planned"
            if self.use_arrow:
                msg.type = 0
                msg.scale.x = self.length
                msg.scale.y = self.width
                msg.scale.z = self.width * 2
            else:
                # Sphere
                msg.type = 2
                msg.scale.x = self.width
                msg.scale.y = self.width
                msg.scale.z = self.width
            msg.id = self.path2_counter
            self.path2_counter = self.path2_counter + 1
            self.pub.publish(msg)
        self.prev_pos2 = end

    def path2_callback(self, msg):
        self.path_callback(msg, False)


if __name__ == '__main__':
    rospy.init_node('path_visualizer', anonymous=True)
    pv = PathVisualizer()
    rospy.spin()
