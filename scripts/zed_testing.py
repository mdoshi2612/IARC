#!/usr/bin/env python3

import cv2
import numpy as np
from time import sleep
from zed import ZED_output as camera

def test():
    c = camera()
    sleep(5)
    print(c.img_depth.shape)



if __name__ == '__main__':
    test()