#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped,TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Int32
#from tf.transformations import quaternion_from_euler
from math import cos,pi

class uav:
	def __init__(self):
		rospy.init_node('offboard_node', anonymous=True)
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.loc_pose)
		rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pose )
		rospy.Subscriber('/mavros/setpoint_raw/target_local',PositionTarget,self.get_lin_acc)
		self.pub= rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		#self.pub2 = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)
		#self.pub3 = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size = 10)
		self.loc = Point()
		self.glob = Point()

		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=1)
		#self.pub_setpoint_attitude=rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped,queue_size=1)
		#self.pub_thrust=rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust,queue_size=1)
		self.acc_x=0
		self.acc_y=0
		self.acc_z=0

	def setmode(self,md):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)

	def takeoff(self, alt):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			mode = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
			response = mode(0,0, self.glob.x, self.glob.y, alt)
			response.success
		except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)

	def setarm(self,av): # input: 1=arm, 0=disarm
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException as e:
			print ("Service call failed: %s" %e)

	def gotopose(self, x, y ,z):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		dist = np.sqrt(((self.loc.x-x)**2) + ((self.loc.y-y)**2) + ((self.loc.z-z)**2))
		while(dist > 0.18):
			self.pub.publish(self.sp)
			dist = np.sqrt(((self.loc.x-x)**2) + ((self.loc.y-y)**2) + ((self.loc.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)

	def getvelBody(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			self.pub2.publish(msg)
			r.sleep()

	def get_lin_acc(self,acc_data):
		#self.acc_x=acc_data.linear_acceleration.x
		#self.acc_y=acc_data.linear_acceleration.y
		#self.acc_z=acc_data.linear_acceleration.z
		self.acc_x=acc_data.acceleration_or_force.x
		self.acc_y=acc_data.acceleration_or_force.y
		self.acc_z=acc_data.acceleration_or_force.z



	def getvelLocal(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(20) # 10hz
		while not rospy.is_shutdown():
			self.pub2.publish(msg)
			r.sleep()

	def circle(self, x, y, z):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 1543
		msg.velocity.x = 0
		msg.velocity.y = -1 
		msg.velocity.z = 0
		msg.acceleration_or_force.x = -0.2
		msg.acceleration_or_force.y = 0
		msg.acceleration_or_force.z = 0
		msg.yaw_rate = -0.2
		self.pub2.publish(msg)
	   	
	   	
	def offboard(self):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 10.0
		for i in range(10):
			self.pub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("GUIDED")

	def loc_pose(self, data):
		self.loc.x = data.pose.position.x
		self.loc.y = data.pose.position.y
		self.loc.z = data.pose.position.z

	def global_pose(self, data):
		self.glob.x = data.latitude 
		self.glob.y = data.longitude  
		self.glob.z = data.altitude  

	def flip(self):
		
		'''print('Starting')
		rospy.sleep(5)
		rate = rospy.Rate(20)
		sr = AttitudeTarget()
		#sr.type_mask = 135
		sr.type_mask = 128
		sr.header.frame_id="map"
		sr.body_rate.x = 0.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0

		sr.thrust = 0.9
		for i in range(20):

			sr.header.stamp=rospy.Time.now()
			#print('stg 1\n')
			self.publish_attitude_thrust.publish(sr)
			
			rate.sleep()
		print('Stage 1 done')
		#sr.header.frame_id="map"
		sr.type_mask = 128
		sr.body_rate.x = 1500.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 0.5
		for i in range(20):

			#print('stg 2\n')
			sr.header.stamp=rospy.Time.now()
			self.publish_attitude_thrust.publish(sr)
			rate.sleep()
		print('Stage 2 done')
		sr.header.frame_id="map"
		sr.type_mask = 128
		sr.body_rate.x = -1500.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 0.5
		for i in range(5):

			#print('final')
			sr.header.stamp=rospy.Time.now()
			self.publish_attitude_thrust.publish(sr)
			rate.sleep()
		print('Roll Complete!!')'''
		#self.setmode("GUIDED_NOGPS")
		print('Starting')
		rospy.sleep(5)
		rate = rospy.Rate(20)
		sr = AttitudeTarget()
		sr.type_mask = 134
		sr.thrust = 0.0
		sr.body_rate.x = 0.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.3
		for i in range(40):
			#print('descend - 0.0',self.acc_x,self.acc_y,self.acc_z)
			self.publish_attitude_thrust.publish(sr)
			print('descend - 0.0 | Z acceleration',self.acc_z)
			rate.sleep()
		print('Stage 1 done')
		
		rospy.sleep(5)
		'''print('descend - 0.0 | Z acceleration',self.acc_z)
		sr.type_mask = 134
		#sr.body_rate.x = 1500.0
		sr.body_rate.x = 0.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 0.5
		for i in range(40):
			#print('Hover - 0.5',self.acc_x,self.acc_y,self.acc_z)
			self.publish_attitude_thrust.publish(sr)
			print('Hover - 0.5 | Z acceleration',self.acc_z)
			rate.sleep()
		print('Stage 2 done')
		
		rospy.sleep(5)
		print('Hover - 0.5 | Z acceleration',self.acc_z)
		sr.type_mask = 134
		# sr.body_rate.x = -1500.0
		sr.body_rate.x = 0.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 1
		for i in range(40):
			
			self.publish_attitude_thrust.publish(sr)
			print('Ascend - 1 | Z acceleration',self.acc_z)
			rate.sleep()
		print('Roll Complete!!')	
		rospy.sleep(5)
		print('Ascend - 1 | Z acceleration',self.acc_z)'''

	




child = uav()
child.setarm(1)
rospy.sleep(2)
child.offboard()
child.takeoff(10)

child.gotopose(0, 0, 10)
rospy.sleep(5)
child.flip()

#child.circle(0,0,7)
# rospy.sleep(2)
# # child.offboard()
#child.gotopose(5, 0, 10)
#child.gotopose(5, 5, 2)
#child.gotopose(0, 0, 5)
rospy.sleep(2)
# r = rospy.Rate(10) # 10hz
# while not rospy.is_shutdown():
# 	child.gotopose(5, 0, 5)
# 	r.sleep()