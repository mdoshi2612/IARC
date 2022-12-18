#!/usr/bin/env python3
from email import header
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import time
import ros_numpy
import numpy as np



# print(x.dtype.names)
# y = ros_numpy.point_cloud2.array_to_pointcloud2(x)
# print(y)




def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions (m)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    # points = points
    # print(points)
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
    # print(itemsize)

    data = points.astype(dtype).tobytes()
    # print(len(data))

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]
    # print(len(fields))

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )
class pc_vis:
    def __init__(self, topic_name = 'pc'):
        self.arr = None
        self.pub = rospy.Publisher('mlcv/' + topic_name, sensor_msgs.PointCloud2,
                                 queue_size=10)

    def push(self, arr, color = np.array([1,1,1,1])):
        self.arr = arr
        self.arr = self.arr.T.copy()
        temp = np.array(np.ones((4, self.arr.shape[1]), dtype = float).T*color*255, dtype = int).T
        # print(temp)
        self.arr = np.append(self.arr, temp, axis= 0).T
        # print(self.arr)
        self.msg = point_cloud(self.arr, '/map')
        self.pub.publish(self.msg)


if __name__ == '__main__':
    # x = np.ones((10,3), dtype=float)
    # x = np.array([[1,1,1],
    #               [2,2,2],
    #               [3,3,3],
    #               [4,4,4],
    #               [5,5,5]
    #               ], dtype=float)
    x = np.array([[0,0,0]])
    rospy.init_node('dragon_curve')
    p = pc_vis()
    p.push(x, color=np.array([0.1, 0, 0.1, 0.5]))
    # exit()
    while True:
        p.push(x, color=np.array([0.1, 0, 1, 1]))
        # p.push(x)
        time.sleep(0.1)

    # print(y)