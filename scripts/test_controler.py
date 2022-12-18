#!/usr/bin/env python3
import zzz as controler
from time import sleep
import numpy as np
import math
import matplotlib.pyplot as plt


def cal_x(y):
    mast_pose = [0, 0]
    x = -1*np.sqrt(25 - (y - mast_pose[1])*(y - mast_pose[1])) + mast_pose[0]
    return x

def go_around(iris_controller):
    mast_pose = [0, 0]
    y = [-4.7, 3]
    y = np.linspace(y[0], y[1], num = 20)
    
    x = cal_x(y)
    #print(x)
    
    iris_controller.toggle_arm(True)
    iris_controller.takeoff(2.0)
    iris_controller.set_offboard_mode()
    i = 1
    x[i] = 4
    y[i] = -3
    x[1] = -1.13
    y[1] = -0.7
    iris_controller.move_to(x[i], y[i], 1.2)
    iris_controller.set_orientation(0, 0, 120)
    #print(math.degrees(math.atan2(mast_pose[1] - y[i],mast_pose[0] - x[i]) - 10))
    iris_controller.set_pose()
    sleep(2)
if __name__ == '__main__':
    try:
            iris_controller = controler.Flight_controller()
    except :
            pass


    go_around(iris_controller)
    