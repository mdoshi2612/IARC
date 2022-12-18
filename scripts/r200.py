#!/usr/bin/env python3
import rospy
import cv2
from copy import copy
import numpy as np
from sensor_msgs.msg import NavSatFix, Image, CameraInfo
from time import sleep
import ros_numpy
from rospy.numpy_msg import numpy_msg

class R200_output:

    def __init__(self):
        self.img_count = 7
        self.img_rgb = None
        self.img_depth = None

        #NODE
        rospy.init_node('R200_processing', anonymous= True)

        #SUBSCRIBER
        self.get_rgb_image = rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_rgb)
        self.get_depth_image = rospy.Subscriber('/camera/depth/image_raw', numpy_msg(Image), self.get_depth)
        self.depth_camera_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.get_info)

    def get_depth(self, depth_data):
        self.depth_data = copy(depth_data)
        self.img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        self.img_depth = ros_numpy.numpify(depth_data)

    def get_rgb(self, rgb_data):
        self.img_rgb = np.frombuffer(rgb_data.data, dtype=np.uint8).reshape(rgb_data.height, rgb_data.width, -1)
        self.img_bgr = self.img_rgb[...,::-1]
        

    def get_info(self, data):
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.fx = 1.0 / data.K[0]
        self.fy = 1.0 / data.K[4]
        self.Px = data.K[0]
        self.Py = self.Px
        
        
    
    def click_rgb_image(self):
        self.get_rgb_image

    def click_depth_image(self):
        self.get_depth_image

    def save_rgb(self):
        self.click_rgb_image()
        rgb_filename = 'drone_img/rgb/rgb_camera_image' + str(self.img_count)  + '.jpeg'
        cv2.imwrite(rgb_filename, self.img_bgr)
        self.img_count += 1
        print(rgb_filename + ' Saved')

    def save_depth(self):
        self.click_depth_image()
        depth_filename = 'drone_img/depth/depth_camera_image' + str(self.img_count)  
        #cv2.imwrite(depth_filename + '.png', self.img_depth)
        np.save(depth_filename + '.npy', self.img_depth)
        self.img_count += 1
        print(depth_filename + ' Saved')




        


if __name__ == '__main__':
    try:
            R200 = R200_output()
    except rospy.ROSInterruptException:
            pass
    R200.click_depth_image()
    R200.click_rgb_image()
    R200.depth_camera_info
    sleep(4)
        