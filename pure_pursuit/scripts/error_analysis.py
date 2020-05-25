#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from pure_pursuit.msg import error_pp
import tf
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

		self.path_pub = rospy.Publisher('/error', error_pp, queue_size=1)
		self.pf_sub = rospy.Subscriber('/pf/viz/odom', Odometry, self.pf_callback)
		self.dg_sub = rospy.Subscriber('/desired_path', PoseStamped, self.dg_callback)

	def pf_callback(self, msg):
		self.pf.pose.pose.position.x = msg.pose.pose.position.x
		self.pf.pose.pose.position.y = msg.pose.pose.position.y

		self.pf.pose.pose.orientation.qx = msg.pose.pose.orientation.x
		self.pf.pose.pose.orientation.qy = msg.pose.pose.orientation.y
		self.pf.pose.pose.orientation.qz = msg.pose.pose.orientation.z
		self.pf.pose.pose.orientation.qw = msg.pose.pose.orientation.w

		pose_quaternion = np.array([qx, qy, qz, qw])
		self.pf_yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
	
	def dg_callback(self, msg):
		x = msg.pose.position.x
		y = msg.pose.position.y

		qx = msg.pose.orientation.x
		qy = msg.pose.orientation.y
		qz = msg.pose.orientation.z
		qw = msg.pose.orientation.w

		pose_quaternion = np.array([qx, qy, qz, qw])
		yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
		
		self.error = abs(self.pf - msg)
		error = error_pp()
		error.theta = abs(self.pf_yaw - yaw)
		error.dist = math.sqrt((self.pf.pose.pose.position.x - x)**2 + (self.pf.pose.pose.position.y - y)**2)
		if error.dist > self.max_error:
			self.max_error = error.dist
		error.max = self.max_error	
		error_to_csv = (error.dist, error.theta, error.max)
		self.error_list.append(error_to_csv)
		self.path_pub.publish(error)

	def save_csv(self):
		print("Saving waypoints...")
		df = pd.DataFrame(self.error_list, columns=['Error', 'theta', 'Max Error'])
		df.to_csv('error.csv')

if __name__ == '__main__':
	rospy.init_node('pp_error', anonymous=True)
	error = Error_analysis()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
