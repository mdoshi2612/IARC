#!/usr/bin/env python3
import rospy
from mlcv.msg import mlcv


class tracker_out:
    def __init__(self):
        self.xyz = [0,0,0]
        self.angle = 0
        self.sense = 0

        rospy.init_node('mlcv', anonymous=True)

        self.mlcv_pub = rospy.Publisher('/mlcv/mlout', mlcv, queue_size = 10)

    def pub(self):
        p = mlcv()
        p.x = 0
        p.y = 1
        p.z = 2
        self.mlcv_pub.publish(p)




if __name__ == "__main__":
    
    t = tracker_out()
    s = rospy.Rate(10)
    while True:
        t.pub()
        s.sleep()




#!/usr/bin/env python3
import rospy
import math

from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped, Vector3Stamped
from time import sleep
from scipy.spatial.transform import Rotation as R
from mlcv.msg import mlcv


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


#from tf.transformations import euler_from_quaternion, quaternion_from_euler
class Flight_controller:
    def __init__(self, cv = False):
        
        #data
        self.gps_lat = 0
        self.gps_long = 0
        self.gps_alt_correction = 535.298913472
        self.img_count = 0

        self.curr_x = 0
        self.curr_y = 0
        self.curr_z = 0
        self.curr_ori_x = 0
        self.curr_ori_y = 0
        self.curr_ori_z = 0.707
        self.curr_ori_w = 0.707
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0

        self.set_x = 0
        self.set_y = 0
        self.set_z = 0
        self.set_ori_x = 0
        self.set_ori_y = 0
        self.set_ori_z = 0.707
        self.set_ori_w = 0.707
        self.set_yaw = 0
        

        
        self.delta = 0.1
        self.delta_z = 0.1
        self.delta_yaw = 0.1
        self.waypoint_number = 0
        self.mast_yaw = math.degrees(-0.831937)


        #NODE
        rospy.init_node('iris_drone', anonymous=True)

        #SUBSCRIBERS
        self.gps_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
        self.get_vel_subscriber = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
        
        if cv:
            self.mlcv = rospy.Subscriber('/mlcv/mlout', mlcv, self.ml_callback)
        

        #PUBLISHERS
        self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10 )
        self.pub_acc = rospy.Publisher('/mavros/setpoint_accel/accel',Vector3Stamped,queue_size= 10)
        
        
        #SERVICES
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo('INIT')

    #Mode setup

    def ml_callback(self, data):
        self.detected = data.detected
        self.mast_x = data.x
        self.mast_y = data.y
        self.mast_z = data.z
        self.mast_angle = 180 - data.angle
        self.mast_sense = data.sense
        self.mast_xyz = [data.x, data.y, data.z]

    def toggle_arm(self, arm_bool):

        rospy.wait_for_service('/mavros/cmd/arming')

        try:
            self.arm_service(arm_bool)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def takeoff(self, t_alt):
        self.gps_subscriber
        sleep(2)
        t_lat = self.gps_lat
        t_long = self.gps_long
        try:
            t_alt_gps = self.gps_alt
            print('gps_alt ' + str(self.gps_alt))
            if self.gps_alt > 1.0:
                print('Drone is in air')
                if self.gps_alt > t_alt:
                    return
                if self.gps_alt < t_alt:
                    t_alt -= self.gps_alt
        except:
            pass
        print(t_lat)
        print(t_long)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')

        try:
            #self.takeoff_service(0.0,0.0,47.3977421, 8.5455945, t_alt)
            self.takeoff_service(0.0,0.0,t_lat, t_long, t_alt)
            print('takeoff done')
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def land(self, t_alt):
        self.gps_subscriber

        t_lat = self.gps_lat
        t_long = self.gps_long

        rospy.wait_for_service('/mavros/cmd/land')

        try:
            self.land_service(0.0,0.0, t_lat, t_long, t_alt)
            rospy.loginfo('LANDING')
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def set_offboard_mode(self):
        rate = rospy.Rate(20)

        rospy.wait_for_service('/mavros/set_mode')

        PS = PoseStamped()

        PS.pose.position.x = self.curr_x
        PS.pose.position.y = self.curr_y
        PS.pose.position.z = self.curr_z
        self.set_orientation_quaternion(self.set_ori_x, self.set_ori_y, self.set_ori_z, self.set_ori_w)
        for i in range (0,100):
            self.publish_pose.publish(PS)
        
        try:
            self.flight_mode_service(0, 'OFFBOARD')
            rospy.loginfo('OFFBOARD')

        except rospy.ServiceException as e:
            rospy.loginfo('OFFBOARD Mode could not be set: ' %e)

    #CALLBACKS

    def gps_callback(self, data):
        self.gps_lat = data.latitude
        self.gps_long = data.longitude
        self.gps_alt = data.altitude
        if self.gps_alt >= 530:
            self.gps_alt -= self.gps_alt_correction

    def get_pose(self, location_data):
        self.curr_x = location_data.pose.position.x
        self.curr_y = location_data.pose.position.y
        self.curr_z = location_data.pose.position.z
        self.curr_ori_x = location_data.pose.orientation.x
        self.curr_ori_y = location_data.pose.orientation.y
        self.curr_ori_z = location_data.pose.orientation.z
        self.curr_ori_w = location_data.pose.orientation.w
        rot_q = location_data.pose.orientation
        self.curr_roll ,self.curr_pitch ,self.curr_yaw= quad_eular(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

    # current velocity at any given time stamp
    def get_vel(self,velocity_data):
        self.v_x = velocity_data.twist.linear.x
        self.v_y = velocity_data.twist.linear.y
        self.v_z = velocity_data.twist.linear.z
        self.w_x = velocity_data.twist.angular.x
        self.w_y = velocity_data.twist.angular.y
        self.w_z = velocity_data.twist.angular.z


        
        

    
    def set_pose(self):
        update_rate = rospy.Rate(20)

        PS = PoseStamped()

        PS.pose.position.x = self.set_x
        PS.pose.position.y = self.set_y
        PS.pose.position.z = self.set_z

        PS.pose.orientation.x = self.set_ori_x
        PS.pose.orientation.y = self.set_ori_y
        PS.pose.orientation.z = self.set_ori_z
        PS.pose.orientation.w = self.set_ori_w

        distance = math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)

        while (distance > self.delta):
            self.publish_pose.publish(PS)
            self.get_pose_subscriber
            distance = math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)
            update_rate.sleep()
        while((self.curr_yaw - self.set_yaw)**2 > self.delta_yaw):
            self.publish_pose.publish(PS)
            self.get_pose_subscriber
            update_rate.sleep()

        self.waypoint_number += 1
        rospy.loginfo('Waypoint reached: ' + str(self.waypoint_number))



    #Testing

    def set_waypoint(self, x,y,z):
        self.set_x = x
        self.set_y = y
        self.set_z = z
    def set_orientation_quaternion(self, x, y, z, w):
        _,_, self.set_yaw = quad_eular(x,y,z,w)
        self.set_ori_x = x
        self.set_ori_y = y
        self.set_ori_z = z
        self.set_ori_w = w

    def set_orientation(self, roll, pitch, yaw):
        self.set_yaw = yaw
        q = eular_quad(roll,pitch, yaw)
        self.set_ori_x = q[0]
        self.set_ori_y = q[1]
        self.set_ori_z = q[2]
        self.set_ori_w = q[3]

    def d_yaw(self, dyaw):
        self.set_orientation(0,0, self.curr_yaw + dyaw)

    def move_wrtDrone(self, dx, dy, dz):
        self.set_x = self.curr_x + dx
        self.set_y = self.curr_y + dy
        self.set_z = 1.2



    def move_to(self, x, y, z):
        self.set_waypoint(x,y,z)
        self.set_pose()

    def test_control(self):
        self.toggle_arm(True)
        self.takeoff(2.0)

        self.set_offboard_mode()
        self.move_to(3,0,2.5)
        self.move_to(0, -2, 2.5)
        sleep(2)

    def calculate_fv(self):
        angle = self.curr_yaw - self.mast_yaw
        return angle

    def getvelBody(self, u, v, w, dalpha, dbeta, dgamma):
        msg = Twist()
        msg.linear.x = u
        msg.linear.y = v 
        msg.linear.z = w
        msg.angular.x = dalpha
        msg.angular.y = dbeta
        msg.angular.z = dgamma
        self.pub_vel.publish(msg)

    def accel_command(self,x_acceleration,y_acceleration,z_acceleration):
        v = Vector3Stamped() 
        v.header.stamp = rospy.Time.now()
        v.header.frame_id = 'map'
        v.vector.x = x_acceleration
        v.vector.y = y_acceleration
        v.vector.z = z_acceleration
        self.pub_acc.publish(v)



if __name__ == '__main__':
    pass
