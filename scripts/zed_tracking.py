#!/usr/bin/env python3
from numpy.lib.function_base import angle
import r200 as r200
import rospy
import numpy as np
import cv2
from time import sleep
import math
import detection as dect
import plane_fitting as plane
import transformation as trans
import argparse
import detection_yolo as yolo
import line_fitting as line
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from mlcv.msg import mlcv
from zed import ZED_output

import matplotlib.pyplot as plt



class tracker_out:
    def __init__(self) -> None:
        #rospy.init_node('mlcvout', anonymous=True)

        self.mlcv_pub = rospy.Publisher('/mlcv/mlout', mlcv, queue_size = 10)

    def start_visualizer(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def pub(self, detected, xyz, angle, sense):
        cvout = mlcv()
        cvout.detected = detected
        cvout.x = xyz[0]
        cvout.y = xyz[1]
        cvout.z = xyz[2]
        cvout.angle = angle
        cvout.sense = sense

        self.mlcv_pub.publish(cvout)
        
        

def tracker_yolo(camera, tiny = True):
    
    TO = tracker_out()
    
    board = yolo.yolo(tiny= tiny)
    
    
    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')
    rate = rospy.Rate(20)
    xyz = [0,0,0]
    angle = 0
    sense = 0
    error_count = 1
    total_count = 0
    print('CV loaded to be used')
    # sleep(10)
    print('getting started')
    while True:
        total_count +=1
        bgr = camera.img_bgr_left[...,::-1]
        depth = camera.img_depth
        Fx = camera.Px
        Fy = camera.Py
        ret,x1,y1,x2,y2=yolo.my_detect(board,bgr)
        center = np.array([(x1 + x2)/2, (y1 + y2)/2])
        c_patch = np.array([(x1 + x2)/2 , (y1*3 + y2)/4], dtype=int)
        if ret == True:
            yolo_out = cv2.rectangle(bgr.astype(np.uint8).copy() ,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),3)
            
            
            kx = int(abs(x2 - x1)/8)
            ky = int(abs(y2 - y1)/12)
            
            if kx < 3:
                kx = 3
            if ky  <2:
                ky = 2

            yolo_out = cv2.rectangle(yolo_out, (c_patch[0]-kx, c_patch[1]-ky), (c_patch[0]+kx +1, c_patch[1]+ky+1), (0,255, 0), 3)
            pc, pc_sense, points = plane.point_cloud(center, np.array(c_patch), depth, Fx, Fy, kx = kx, ky = ky)
            t = True
            avg_z = np.sum(pc.T[2])/pc.T[2].shape[0]
            x_center = (center[0] - bgr.shape[1]/2)*avg_z/Fx
            y_center = (center[1] - bgr.shape[0]/2)*avg_z/Fy
            xyz = [x_center, y_center, avg_z]
            # try:
            #     sense = line.best_line(pc_sense)
            #     sense = -1*sense[1]/sense[0]
            #     m = plane.face_vector(pc)
            #     a ,b, c, d = m
            #     angle = trans.angle_with_z([a, b ,c])
            #     #print(angle)
            #     #print(sense)
            #     TO.pub(1, xyz, angle, sense)
            # except:
            #     print('Error in plane finding '+ str(round(error_count/total_count, 3)) + ' kx: ' + str(kx) + ' ky: ' + str(ky))
            #     error_count += 1
            #     TO.pub(0, xyz, angle, sense)
            try:
                sense, best_fit_found = line.best_line(pc_sense)
                if best_fit_found:
                    sense = -1*sense[1]/sense[0]
                else:
                    sense = sense0

                m, best_fit_found = plane.face_vector(pc)
                if best_fit_found:
                    a ,b, c, d = m
                    angle = trans.angle_with_z([a, b ,c])
                #print(angle)
                #print(sense)
                sense0 = sense
                TO.pub(1, xyz, angle, sense)
            except:
                print('Error in plane finding')
                TO.pub(0, xyz, angle, sense)
            
            #print(xyz)

            

            cv2.imshow('YOLO output', yolo_out)
            rate.sleep()
            
        else:
            cv2.imshow('YOLO output', bgr)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        





















if __name__ == '__main__':
    tiny = False
    parser = argparse.ArgumentParser(description='Image count')
    parser.add_argument('algo', type=str, default='yolo',  help='Tracking algorithm')
    args = parser.parse_args()
    try:
        camera = ZED_output()
    except rospy.ROSInterruptException:
            pass

    sleep(5)
    if args.algo == 'yolo':
        tracker_yolo(camera, tiny= tiny)
    else:
        pass
