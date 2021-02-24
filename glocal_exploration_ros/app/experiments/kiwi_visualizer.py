#!/usr/bin/env python

# Python
import csv
import datetime
import os
import re
import shutil
import sys

# ros
import rospy
from visualization_msgs.msg import Marker
import numpy as np
from matplotlib import pyplot as plt


class KiwiVisualizer(object):
    def __init__(self):
        # Params
        self.count = 0

        # ROS
        self.path_sub = rospy.Subscriber('~path_in', Marker,
                                         self.path_callback)
        self.path_sub2 = rospy.Subscriber('~path_in2', Marker,
                                          self.path_callback)
        self.path_pub = rospy.Publisher('path_out', Marker, queue_size=100)

    def path_callback(self, msg):
        msg.scale.x = 0.2
        msg.color.a = 1
        msg.color.r = 0
        msg.color.g = 0
        msg.color.b = 0
        self.path_pub.publish(msg)
        self.count = self.count + 1
        print("Counted %i path segments." % self.count)


if __name__ == '__main__':
    rospy.init_node('kiwi_visualizer')
    vis = KiwiVisualizer()
    rospy.spin()
