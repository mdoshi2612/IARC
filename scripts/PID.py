#!/usr/bin/env python3
import rospy
import numpy as np
class PID:
    def __init__(self, Kp=2, Ki=0.0, Kd=3, maxI=10, maxOut=2, frequency = 20):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.frequency = frequency

        self.target = 0
        self.output = 0
        self.error = 0
        self.maxI = maxI
        self.maxOut = maxOut
        self.reset()

    def update(self, rel_position, rel_vel):
        self.error = rel_position

        p = self.error
        d = rel_vel

        output = np.array((self.Kp * p  + self.Kd * d))
        if(self.maxOut is not None):
            output[output > self.maxOut] = self.maxOut
            output[output < -self.maxOut] = -self.maxOut
        
        return output

    def setKp(self, Kp):
        self.Kp = Kp

    def setKi(self, Ki):
        self.Ki = Ki

    def setKd(self, Kd):
        self.Kd = Kd

    def setMaxI(self, maxI):
        self.maxI = maxI

    def reset(self):
        self.target = 0.0
        self.error = 0.0
        self.state = 0.0
        self.intError = 0.0
        self.lastError = 0.0
        self.output = 0.0
    
    # def plot(self):
