#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import  Image, CameraInfo
from time import sleep
import time

from rospy.numpy_msg import numpy_msg
from copy import copy


import cv2

#(msg_types={'geometry_msgs/PoseStamped': 'd3812c3cbc69362b77dc0b19b345f8f5', 
# 'sensor_msgs/Image': '060021388200f6f0f447d0fcd9c64743', 
# 'sensor_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2'}, 
# topics=
# {'/zed2/zed_node/depth/depth_registered': TopicTuple(msg_type='sensor_msgs/Image', message_count=487, connections=1, frequency=10.498134676249629), 
# '/zed2/zed_node/imu/data': TopicTuple(msg_type='sensor_msgs/Imu', message_count=6485, connections=1, frequency=425.3426630159213), 
# '/zed2/zed_node/left/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=418, connections=1, frequency=10.00494724764624), 
# '/zed2/zed_node/pose': TopicTuple(msg_type='geometry_msgs/PoseStamped', message_count=628, connections=1, frequency=14.898936831523496), 
# '/zed2/zed_node/rgb/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=409, connections=1, frequency=10.073562417815992), 
# '/zed2/zed_node/right/image_rect_color': TopicTuple(msg_type='sensor_msgs/Image', message_count=366, connections=1, frequency=10.072751986666699)})

class ZED_output:

    def __init__(self):
        self.img_count = 7
        self.img_rgb = None
        self.img_depth = None
        self.Px = 700.819
        self.Py = 700.819
        self.count_bgr = 0
        self.count_bgr_r = 0
        self.count_bgr_l = 0
        self.count_depth = 0
        self.t0 = time.time()


        #NODE
        

        #SUBSCRIBER
        #RGBA
        self.get_rgb_image_left = rospy.Subscriber('/zed2/zed_node/left/image_rect_color', Image, self.get_rgb_left)
        #DEPTH
        self.get_depth_image = rospy.Subscriber('/zed2/zed_node/depth/depth_registered', Image, self.get_depth)
        #intrinsic parameters
        self.depth_camera_info = rospy.Subscriber('/zed2/zed_node/depth/camera_info', CameraInfo, self.get_info)

    def get_depth(self, depth_data):
        self.depth_data = copy(depth_data)
        # t0 = time.time()
        # # self.img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        # img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        # img = np.array(img_depth).copy()
        # img = np.nan_to_num(img)
        # img[img > 20] = 20
        # img[img == np.inf] = 20
        # img = np.reshape(img, (img.shape[0], img.shape[1]))
        # self.img_depth = img.copy()
        # #np.save('zed_data/d'+str(self.count_depth), self.img_depth)
        # #self.count_depth += 1
        # #self.img_depth = ros_numpy.numpify(depth_data)
        # # print('frame grab '+str(1/(t0 - self.t0)))
        # self.t0 = t0


    def get_rgb_left(self, rgb_data):
        self.img_rgb_left = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)[:,:,0:3]
        self.img_bgr_left = self.img_rgb_left[...,::-1]
        # print('hmm')
        #cv2.imwrite('zed_data/zl'+str(self.count_bgr_l)+ '.jpeg', self.img_bgr_left)
        #self.count_bgr_l += 1
        

    def get_info(self, data):
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.fx = 1.0 / data.K[0]
        self.fy = 1.0 / data.K[4]
        
        




        


if __name__ == '__main__':
    rospy.init_node('Zed2_processing', anonymous= True)
    try:
            camera = ZED_output()
    except rospy.ROSInterruptException:
            pass
    sleep(120)
    

    #np.save('depth.npy', np.array(camera.img_depth))
    
