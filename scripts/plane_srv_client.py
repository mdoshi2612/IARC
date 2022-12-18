#!/usr/bin/env python3
from tokenize import Double
from mlcv.srv import plane
import rospy
# from std_msgs.msg import Float64
from zed import ZED_output
from copy import copy
from time import sleep
import time




class plane_client:
    def __init__(self, full_cloud = False):
        self.full_cloud = full_cloud
        rospy.wait_for_service('plane_seg')
        self.plane_seg = rospy.ServiceProxy('plane_seg', plane)
        if full_cloud:
            rospy.wait_for_service('depth_to_cloud')
            self.plane_seg_full = rospy.ServiceProxy('depth_to_cloud', plane)


    def __call__(self, depth, height, width, cx, cy, fx, fy, x1, y1, x2, y2, debug = False, find_plane = True):
        t0 = time.time()
        out = []
        try:
            temp = self.plane_seg( depth = depth, height = height, width = width, cx = cx, cy = cy, fx = fx, fy = fy, 
                x1 = x1, y1 = y1, x2 = x2, y2 = y2, debug =debug, find_plane = find_plane)
            if self.full_cloud:
                whatever = self.plane_seg_full( depth = depth, height = height, width = width, cx = cx, cy = cy, fx = fx, fy = fy, 
                x1 = 0, y1 = 0, x2 = width, y2 = height, debug =debug, find_plane = False)
            out.append(True)
            out.append(temp)
            
        except:
            out.append(False)
        t1 = time.time()
        print('fps : ' +str(1/(t1 - t0)))
        return out

        # except:
        #     return 'fuck'





if __name__ == '__main__':
    rospy.init_node('plane_seg_client', anonymous= True)
    camera = ZED_output()
    pc = plane_client(True)
    sleep(5)
    depth = copy(camera.depth_data.data)
    # bbox = Float64()
    bbox = int(10)
    # bbox = [0,0,10,10]
    # print(depth.data)
    coff = pc(depth,camera.depth_data.height, camera.depth_data.width,camera.cx, camera.cy, camera.fx, camera.fy, 10, 10, 100, 100)
    print(coff)
    # exit()
    for i in range(0, 100):
        depth = copy(camera.depth_data.data)
        coff = pc(depth,camera.depth_data.height, camera.depth_data.width,camera.cx, camera.cy, camera.fx, camera.fy, 10, 10, 30, 30, debug= True)
        sleep(1)
        print(coff)
