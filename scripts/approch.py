#!/usr/bin/env python3
import controller_ardu as controler
from time import sleep
import numpy as np
import math
import matplotlib.pyplot as plt
import PID
import rospy

'lol just checking git'
def next_move(xyz, angle, clockwise = True):
    #print(clockwise)
    th_angle = 6*np.tanh(180 - angle)
    th_dist = 2.3
    fact = 0.4
    r = math.sqrt(np.sum(np.array(xyz)*np.array(xyz))) 
    dy_dr = np.tanh(r - th_dist)*fact
    if clockwise:
        if angle > 5 or r>2:
            dx = 1*r*(math.cos(math.radians(angle + th_angle)) - math.cos(math.radians(angle))) #*1.5
            dy = 1*r*(math.sin(math.radians(angle + th_angle)) - math.sin(math.radians(angle))) + dy_dr
            return dx, dy
        else:
            return 0,0

    else: 
        if angle > 5 or r>2:
            dx = -r*(math.cos(math.radians(angle + th_angle)) - math.cos(math.radians(angle))) #*1.5
            dy = r*(math.sin(math.radians(angle + th_angle)) - math.sin(math.radians(angle))) + dy_dr
            return dx, dy
        else:
            return 0,0

def d_yaw_cal(xyz):
    return math.degrees(math.atan2(xyz[2], xyz[0])) - 90




class follow:
    def __init__(self, xyz0) -> None:
        self.pidx = PID.PID()
        self.pidx.Kp = 0.8
        self.pidx.Kd = 1
        self.pidz = PID.PID()
        self.pidy = PID.PID()
        self.pidy.Kp = 0.8
        self.pidy.Kd = 0.8
        self.frequency = 10
        self.xyz0 = xyz0

    def relative_vel(self):
        self.rel_posx = self.xyz1[0]
        self.rel_posz = self.xyz1[2]
        self.rel_posy = self.xyz1[1]
        self.rel_velx = (self.xyz1[0] - self.xyz0[0])*self.frequency
        self.rel_vely = (self.xyz1[1] - self.xyz0[1])*self.frequency
        self.rel_velz = (self.xyz1[2] - self.xyz0[2])*self.frequency

    def update(self, xyz1):
        self.xyz1 = xyz1
        self.relative_vel()
        self.xyz0 = self.xyz1
        self.outx = self.pidx.update(self.rel_posx, self.rel_velx) # acceleration in x and z
        self.outz = self.pidz.update(self.rel_posz, self.rel_velz)
        self.outy = self.pidy.update(self.rel_posy, self.rel_vely)


# def clear_rotation_offset():

def camera_to_local(x,y,z):
    # return z, -x,-y 
    return x, z, -y

def local_to_camera(x,y,z):
    # return -y, -z, x
    return x, -z, y 

def search(ic):
    rate = rospy.Rate(10)
    ic.move_wrtDrone_fixedaxis(0, 0, 0)
    for i in range(0, 1000):
        ic.d_yaw(2)
        ic.pub_next_pose()
        rate.sleep()
        if ic.got_mldata > 0:
            return

if __name__ == '__main__':
    debug = True
    full_run = False
    ic = controler.Flight_controller(cv= True)
    last_ml_data_pt = 0
    error_range = 10 #degree
    count_failsafe = 0
    
    r = rospy.Rate(10) # rate of publishing acceleration commands
    sleep(2)
    ic.set_offboard_mode()
    if full_run:
        print(ic.curr_x)
        ic.toggle_arm(1)
        sleep(0.5)
        ic.set_offboard_mode()
        sleep(1.5)
        ic.takeoff(1.2)
        sleep(10)
        while ic.curr_z < 1:
            r.sleep()
        print('Now searcing the mast')
        search(ic)
 
    # ic.set_orientation(0,0,135)
    # ic.set_waypoint(0,0,1.2)
    # print('sd')
    # ic.set_pose()
    # print('df')
        print('ML data recived')
    while True:
        if ic.got_mldata > 0:
            while ic.mast_angle > error_range or count_failsafe < 10:
                if ic.mast_angle < error_range:
                    count_failsafe += 1
            # print('while loop running')
                if ic.got_mldata > last_ml_data_pt:
                    last_ml_data_pt = ic.got_mldata
                    

                    
                    if debug:
                        print(ic.got_mldata, last_ml_data_pt)
                        print('detected')
                        print('mast angle : ',ic.mast_angle)
                    if ic.mast_sense < 0:
                        clockwise = True
                    else:
                        clockwise = False
                    # print('mast angle = '+str(ic.mast_angle))
                    dx, dy = next_move(ic.mast_xyz, ic.mast_angle, clockwise)
                    dyaw = d_yaw_cal(ic.mast_xyz)
                    yaw = math.radians(ic.curr_yaw) # get the current yaw of the drone
                    rotation_matrix = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])

                    new_dx, new_dy = np.matmul(rotation_matrix, np.array([dx, dy]))
                    
                    ic.d_yaw(dyaw=dyaw)
                    ic.move_wrtDrone_fixedaxis(new_dy, new_dx, 0)
                    ic.pub_next_pose()
                    r.sleep()
            # print ("TRACKING NOW!!")
            
                # ic.set_waypoint(ic.curr_x,ic.curr_y,ic.curr_z)
                # ic.pub_next_pose()

            ic.land(0)
            sleep(2)
            exit()
            # foll = follow(ic.mast_xyz)
            # while True:
            #     if ic.detected:
            #         foll.update(ic.mast_xyz)
            #         print(foll.outx,foll.outy,foll.outz)
            #         accel_x = foll.outx # left and right
            #         # accel_z = foll.outz
            #         accel_z = 0 # in and out
            #         accel_y = -foll.outz # up and down

            #         print('cam frame accel :',accel_x,accel_y)
            #         # yaw = math.radians(ic.curr_yaw)
            #         yaw = -1*math.radians(ic.curr_yaw) # get the current yaw of the drone
            #         rotation_matrix = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]]) # define the rotation matrix
            #         accel_x,accel_y,accel_z = camera_to_local(accel_x,accel_y,accel_z) # get the acceleration of the drone in the real non-offset global frame
            #         accel_old = np.array([accel_x,accel_y])
            #         accel_new = np.matmul(rotation_matrix,accel_old)
            #         #print ("X-acceleration:",[accel_new,accel_z])
            #     else:
            #         accel_new = [0,0]
            #         accel_z = 0
            #     print('ground frame accel :',accel_new[0],accel_new[1], accel_z)
            #     #ic.accel_command(accel_new[0],accel_new[1], accel_z)
            #     r.sleep()

        # else:
        #     pass