#!/usr/bin/env python3
import rospy
import numpy as np
from time import sleep
import cv2
import time

from zed import ZED_output
from detection_yolo import yolo, my_detect


class zed_yolo:
    def __init__(self):
        self.yolo = yolo(tiny = True)
        self.camera = ZED_output()
        self.bgr = None

    def detect(self):
        self.bgr = self.camera.img_bgr_left
        self.yolo_out = my_detect(self.yolo, self.bgr)



if __name__ == '__main__':
    z = zed_yolo()
    sleep(5)
    while True:
        tic = time.time() 
        z.detect()
        tac = time.time()
        print(1/(tac - tic))
        # print(z.yolo_out)
