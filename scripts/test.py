#!/usr/bin/env python3
from email import header
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import time
import ros_numpy
import numpy as np

def callback(data):
    msg = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    print(msg)


if __name__ == '__main__':
    rospy.init_node('dragon_curve_sub')
    sub = rospy.Subscriber('mlcv/pc', PointCloud2, callback)
    rospy.spin()