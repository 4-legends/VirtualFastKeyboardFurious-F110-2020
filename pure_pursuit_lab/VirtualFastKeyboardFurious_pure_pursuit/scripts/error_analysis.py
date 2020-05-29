#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
import tf
import numpy as np
import pandas as pd 
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class Error_analysis:
	def __init__(self):
		self.pf = Odometry()
		self.pf.pose.pose.position.x = 0
		self.pf.pose.pose.position.y = 0
		self.error_list = []
		self.max_error = 0
		rospy.on_shutdown(self.save_csv)
		self.current_pos = Odometry()
		self.current_pos.pose.pose.position.x = 0
		self.current_pos.pose.pose.position.y = 0
		self.pf_sub = rospy.Subscriber('/pf/pose/odom', Odometry, self.dg_callback)
		self.dg_sub = rospy.Subscriber('/desired_path', PoseStamped, self.pf_callback)

	def pf_callback(self, msg):
		x = self.pf.pose.pose.position.x
		y = self.pf.pose.pose.position.y
		self.pf.pose.pose.position.x = msg.pose.position.x
		self.pf.pose.pose.position.y = msg.pose.position.y
		if x != self.pf.pose.pose.position.x: 
			self.current_pos.pose.pose.position.x = x
		if y != self.pf.pose.pose.position.y:
			self.current_pos.pose.pose.position.y = y
			

			

		#self.pf.pose.pose.orientation.x = msg.pose.orientation.x
		#self.pf.pose.pose.orientation.y = msg.pose.orientation.y
		#self.pf.pose.pose.orientation.z = msg.pose.orientation.z
		#self.pf.pose.pose.orientation.w = msg.pose.orientation.w
		#qx = self.pf.pose.pose.orientation.x
		#qy = self.pf.pose.pose.orientation.y
		#qz = self.pf.pose.pose.orientation.z
		#qw = self.pf.pose.pose.orientation.w		
		#pose_quaternion = np.array([qx, qy, qz, qw])
		#self.pf_yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
		
	
	def dg_callback(self, msg):
		
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		 
		#qx = msg.pose.pose.orientation.x
		#qy = msg.pose.pose.orientation.y
		#qz = msg.pose.pose.orientation.z
		#qw = msg.pose.pose.orientation.w

		#pose_quaternion = np.array([qx, qy, qz, qw])
		#yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
		error = Float64()
		base = math.sqrt((self.current_pos.pose.pose.position.x - self.pf.pose.pose.position.x)**2 + (self.current_pos.pose.pose.position.y - self.pf.pose.pose.position.y)**2)
		
		a = self.current_pos.pose.pose.position.x - x
		b = self.current_pos.pose.pose.position.x - self.pf.pose.pose.position.x
		c = self.current_pos.pose.pose.position.y - y
		d = self.current_pos.pose.pose.position.y - self.pf.pose.pose.position.y
		area = 0.5*(a*d - b*c)
		error = (2*area)/(base)
		
		if error > self.max_error:
			self.max_error = error
		max_error = self.max_error	
		look = rospy.get_param('LOOKAHEAD_DISTANCE')
		error_to_csv = (max_error, error)
		self.error_list.append(error_to_csv)
		rospy.loginfo("Current_pos = (%f,%f) Next_Goal = (%f, %f) Car_Pos = (%f, %f) Error = %f", self.current_pos.pose.pose.position.x, self.current_pos.pose.pose.position.y, self.pf.pose.pose.position.x, self.pf.pose.pose.position.y, x, y, error)
		

	def save_csv(self):
		print("Saving waypoints...")
		df = pd.DataFrame(self.error_list, columns=['Max Error', 'Error'])
		df.to_csv('error.csv')

if __name__ == '__main__':
	rospy.init_node('pp_error', anonymous=True)
	error = Error_analysis()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
