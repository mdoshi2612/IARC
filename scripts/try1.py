#!/usr/bin/env python3
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import SetMode, CommandBool
class mavcon:
	def __init__(self):
		rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.loc_pose)
		self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=50)
		self.pubr = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
		self.pt = Point()

	def setmode(self,md):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException:
			print ("Service call failed:")

	def setarm(self,av): # input: 1=arm, 0=disarm
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException:
			print ("Service call failed:")

	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z
		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.08):
			self.pub.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		print('Reached ',x,y,z)

	def offboard(self):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 0.0
		for i in range(5):
			self.pub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("OFFBOARD")

	def flip(self):
		rate = rospy.Rate(20)
		sr = AttitudeTarget()
		sr.type_mask = 135
		sr.thrust = 0.9
		for i in range(20):
			self.pubr.publish(sr)
			rate.sleep()
		print('Stage 1 done')
		sr.type_mask = 134
		sr.body_rate.x = 1500.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 0.5
		for i in range(20):
			self.pubr.publish(sr)
			rate.sleep()
		print('Stage 2 done')
		sr.type_mask = 134
		sr.body_rate.x = -1500.0
		sr.body_rate.y = 0.0
		sr.body_rate.z = 0.0
		sr.thrust = 0.5
		for i in range(5):
			self.pubr.publish(sr)
			rate.sleep()
		print('Roll Complete!!')
		# sr1 = AttitudeTarget()
		# sr1.type_mask = 6
		# sr1.body_rate.x = 0.0
		# sr1.thrust = 0.8
		# sr1.orientation.x = 0.0
		# sr1.orientation.y = 0.0
		# sr1.orientation.z = 0.0
		# sr1.orientation.w = 0.0
		# for i in range(20):
		# 	self.pubr.publish(sr1)
		# 	rate.sleep()
		# sr.type_mask = 132
		# sr.body_rate.x = 0.0
		# sr.thrust = 0.3
		# for i in range(60):
		# 	self.pubr.publish(sr)
		# 	rate.sleep()


	def loc_pose(self,data):
		self.pt.x = data.pose.position.x
		self.pt.y = data.pose.position.y
		self.pt.z = data.pose.position.z