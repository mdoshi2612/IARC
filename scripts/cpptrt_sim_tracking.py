#!/usr/bin/env python3
from numpy.lib.function_base import angle
from copy import copy
import r200 as r200
import rospy
import numpy as np
import cv2
from time import sleep, time
import math
import detection as dect
import plane_fitting as plane
import transformation as trans
import argparse
import yolo_trt as yolo
import line_fitting as line
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from mlcv.msg import mlcv
from plane_srv_client import plane_client

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
        
def calc(out):
    xyz = [out.x, out.y, out.z]
    coff = np.array([out.a, out.b, out.c, out.d])
    if coff[3] < 0:
        coff = -1*coff
    angle = trans.angle_with_z(coff[0:3])
    sense = coff[0]*coff[2]
    return xyz, angle, sense

def tracker_yolo(camera, tiny = False):
    
    TO = tracker_out()
    trt_engine = yolo.trt_yolo(tiny = False)
    plane_segmentation = plane_client(full_cloud = False)
    fps = 1
    rate = rospy.Rate(fps)
    print('CV loaded to be used')
    print('getting started')
    while True:
        t0 = time()
        # sleep(1)
        bgr = camera.img_rgb[...,::-1].copy()
        depth = copy(camera.depth_data.data)
        ret,x1,y1,x2,y2=trt_engine(bgr)
        if ret == True:
            yolo_out = cv2.rectangle(bgr.astype(np.uint8).copy() ,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),3)
            
            x1_new = int((2*x1 + x2)/3)
            x2_new = int((x1 + 2*x2)/3)

            y1_new = int((5*y1 + y2)/6)
            y2_new = int((4*y1 + 2*y2)/6)



            


            out = plane_segmentation(depth, camera.depth_data.height, camera.depth_data.width,camera.cx, 
            camera.cy, camera.fx, camera.fy,x1_new ,y1_new ,x2_new ,y2_new , debug = False, find_plane = True)
            # print(out)
            # xyz = [out.x, out.y, out.z]
            # kx = int(abs(x2 - x1)/8)
            # ky = int(abs(y2 - y1)/12)
            xyz = [0, 0, 0]
            angle = 0
            sense = 0
            if out[0]:
                if out[1].if_found:
                    xyz, angle, sense = calc(out[1])
                    print("ANGLE:", angle)
                    print ("SENSE:",sense)
                    TO.pub(1, xyz, angle, sense)
                else:
                    print('Error in plane finding')
                    TO.pub(0, xyz, angle, sense)

            else:
                print('Error in plane finding')
                TO.pub(0, xyz, angle, sense)
        rate.sleep()
        t1 = time()
        print(int(1/(t1 - t0)))            
            # if kx < 3:
            #     kx = 3
            # if ky  <2:
            #     ky = 2

            # yolo_out = cv2.rectangle(yolo_out, (c_patch[0]-kx, c_patch[1]-ky), (c_patch[0]+kx +1, c_patch[1]+ky+1), (0,255, 0), 3)
            # pc, pc_sense, points = plane.point_cloud(center, np.array(c_patch), depth, Fx, Fy, kx = kx, ky = ky)
            # t = True
            # # print(pc)
            # # print(pc.shape)
            # # pc_mast, _, _ = plane.point_cloud(center, np.array(c_patch), depth, Fx, Fy, kx = abs(int((x2 - x1)/2)), ky = abs(int((y2- y1)/2)))
            # # point_cloud_vis_mast.push(pc_mast, color=np.array([1,1,1,0.1]))
            # point_cloud_vis.push(pc, color=np.array([1,1,0.1,1]))
            
            # avg_z = np.sum(pc.T[2])/pc.T[2].shape[0]
            # x_center = (center[0] - bgr.shape[1]/2)*avg_z/Fx
            # y_center = (center[1] - bgr.shape[0]/2)*avg_z/Fy
            # xyz = [x_center, y_center, avg_z]
            # # try:
            # #     sense = line.best_line(pc_sense)
            # #     sense = -1*sense[1]/sense[0]
            # #     m = plane.face_vector(pc)
            # #     a ,b, c, d = m
            # #     angle = trans.angle_with_z([a, b ,c])
            # #     #print(angle)
            # #     #print(sense)
            # #     TO.pub(1, xyz, angle, sense)
            # # except:
            # #     print('Error in plane finding '+ str(round(error_count/total_count, 3)) + ' kx: ' + str(kx) + ' ky: ' + str(ky))
            # #     error_count += 1
            # #     TO.pub(0, xyz, angle, sense)
            # try:
            #     sense, best_fit_found = line.best_line(pc_sense)
            #     if best_fit_found:
            #         sense = -1*sense[1]/sense[0]
            #     else:
            #         sense = sense0

            #     m, best_fit_found = plane.face_vector(pc)
            #     if best_fit_found:
            #         a ,b, c, d = m
            #         angle = trans.angle_with_z([a, b ,c])
            #     #print(angle)
            #     #print(sense)
            #     sense0 = sense
            #     TO.pub(1, xyz, angle, sense)
            # except:
            #     print('Error in plane finding')
            #     TO.pub(0, xyz, angle, sense)
            
            #print(xyz)

            

    #         cv2.imshow('YOLO output', yolo_out)
    #         # rate.sleep()
            
    #     else:
    #         cv2.imshow('YOLO output', bgr)
    #     if cv2.waitKey(25) & 0xFF == ord('q'):
    #         break
    #     t1 = time()

    #     # print(str(1/(t1 - t0)) + ' fps output')
    # cv2.destroyAllWindows()
        





















if __name__ == '__main__':
    tiny = False
    try:
        camera = r200.R200_output()
    except rospy.ROSInterruptException:
            pass

    sleep(5)
    
    tracker_yolo(camera, tiny= tiny)
    