#!/usr/bin/env python

# ros
import rospy
from sensor_msgs.msg import PointCloud2

# Python
import math


class PointcloudCompleter:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.voxel_size = rospy.get_param('~voxel_size', 0.2)  # m
        self.range = rospy.get_param('~range', 6.5)  # m
        self.horizontal_fov = rospy.get_param('~horizontal_fov', 360) / 180 * math.pi  # deg->rad
        self.vertical_fov = rospy.get_param('~vertical_fov', 45) / 180 * math.pi  # deg->rad

        # ROS
        self.pub = rospy.Publisher("~cloud_in", PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber("cloud_out", PointCloud2, self.callback, queue_size=1)

    def callback(self, msg_in):
        range_img = np.ones((self.horizontal_fov * self.range, self.vertical_fov * self.range)) * self.range

        data = np.transpose(np.vstack((x, y, z, rgb)))

        msg = PointCloud2()
        msg.header = msg_in.header
        msg.width = data.shape[0]
        msg.height = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.float32(data).tostring()
        self.pub.publish(msg)

    def img_to_3d(self, u, v):
        return 0

    def threed_to_img(self, x, y, z):
        return 0


if __name__ == '__main__':
    rospy.init_node('pointcloud_completer', anonymous=True)
    pc = PointcloudCompleter()
    rospy.spin()
