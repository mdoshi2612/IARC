#!/usr/bin/env python3
import rospy
import time
import controler
import PID

class velocity_controller:
    def __init__(self):
        self.x_controller = PID()
        self.y_controller = PID()
        self.z_controller = PID()
        self.yaw_controller = PID()
        
    
    def control(self, state, target):
        self.vel_x = self.x_controller.update(state[0],target[0])
        self.vel_y = self.y_controller.update(state[1],target[1])
        self.vel_z = self.z_controller.update(state[2],target[2])
        self.dyaw = self.yaw_controller.update(state[3],target[3])
        
