#!/usr/bin/env python3
import rospy
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def invert_output(func):
    def decorator(roll, pitch, yaw):
        x,y,z,w = func(roll, pitch, yaw)
        return x, y, -1*z, -1*w
    return decorator

def invert_input(func):
    def decorater(x,y,z,w):
        roll, pitch, yaw = func(x, y, -1*z, -1*w)
        return roll, pitch, yaw
    return decorater

@invert_input
def quad_eular(x, y, z, w):  
    r = R.from_quat([x ,y,z ,w])
    curr_roll ,curr_pitch ,curr_yaw=r.as_euler('xyz', degrees=True)
    return curr_roll, curr_pitch, curr_yaw

@invert_output
def eular_quad(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, math.radians(yaw)])
    x,y,z,w = r.as_quat()
    return x,y,z,w

def set_pt(data):
    temp = data.pose.pose.position
    x, y, z = temp.x, temp.y, temp.z
    temp = data.pose.pose.orientation
    r, p, y = quad_eular(temp.x, temp.y, temp.z, temp.w)
    print(x, y, z, r, p, y)


if __name__ == '__main__':
    rospy.init_node('Obs_pose', anonymous= True)
    pub_setpoint_data = rospy.Subscriber('/mavros/global_position/local', Odometry, set_pt)
    rospy.spin()