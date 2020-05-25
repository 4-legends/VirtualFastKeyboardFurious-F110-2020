#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
#from pure_pursuit.msg import error_pp
import tf
import numpy as np
import pandas as pd 
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class Error_analysis:
	def __init__(self):
		self.pf = Odometry()
		self.error_list = []
		self.max_error = 0
		self.errror = 0
		self.pf_yaw = 0
		rospy.on_shutdown(self.save_csv)

		self.error_pub = rospy.Publisher('/error', Float64, queue_size=1)
		self.max_error_pub = rospy.Publisher('/max_error', Float64, queue_size=1)
		self.pf_sub = rospy.Subscriber('/pf/pose/odom', Odometry, self.dg_callback)
		self.dg_sub = rospy.Subscriber('/desired_path', PoseStamped, self.pf_callback)

	def pf_callback(self, msg):
		self.pf.pose.pose.position.x = msg.pose.position.x
		self.pf.pose.pose.position.y = msg.pose.position.y

		self.pf.pose.pose.orientation.x = msg.pose.orientation.x
		self.pf.pose.pose.orientation.y = msg.pose.orientation.y
		self.pf.pose.pose.orientation.z = msg.pose.orientation.z
		self.pf.pose.pose.orientation.w = msg.pose.orientation.w
		qx = self.pf.pose.pose.orientation.x
		qy = self.pf.pose.pose.orientation.y
		qz = self.pf.pose.pose.orientation.z
		qw = self.pf.pose.pose.orientation.w		
		pose_quaternion = np.array([qx, qy, qz, qw])
		self.pf_yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
	
	def dg_callback(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		qx = msg.pose.pose.orientation.x
		qy = msg.pose.pose.orientation.y
		qz = msg.pose.pose.orientation.z
		qw = msg.pose.pose.orientation.w

		pose_quaternion = np.array([qx, qy, qz, qw])
		yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
		#self.error = abs(self.pf - msg)
		error = Float64()
		theta = abs(self.pf_yaw - yaw)
		error = math.sqrt((self.pf.pose.pose.position.x - x)**2 + (self.pf.pose.pose.position.y - y)**2)
		if error > self.max_error:
			self.max_error = error
		max_error = self.max_error	
		look = rospy.get_param('LOOKAHEAD_DISTANCE')
		error_to_csv = (max_error, error)
		self.error_list.append(error_to_csv)
		self.error_pub.publish(error)
		self.max_error_pub.publish(self.max_error)

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
