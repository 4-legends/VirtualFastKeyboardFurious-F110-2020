#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import copy
#############
# CONSTANTS #
#############
global LOOKAHEAD_DISTANCE
LOOKAHEAD_DISTANCE = 1.5 # meters

global previous_goal
previous_goal = None

ANGLE_LEVEL_1 = 10.0
SPEED_LEVEL_1 = 1
ANGLE_LEVEL_2 = 20.0
SPEED_LEVEL_2 = 0.75
SPEED_LEVEL_3 = 0.5

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
rospy.init_node('pure_pursuit')
tf_listener = tf.TransformListener() #use tf tree for pose transforms

#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /odom.
# Runs pure pursuit and publishes velocity and steering angle.


def find_goal_point(x,y):
    global LOOKAHEAD_DISTANCE
    min_dist = 100000
    status = False
    for i in range(len(path_points)):
        wx,wy,wz = path_points[i] 
        euclid_dist = dist((x, y), (wx,wy))

        if euclid_dist >= LOOKAHEAD_DISTANCE and euclid_dist<min_dist: #base case
            min_dist = euclid_dist
            closest_point = path_points[i] 
            status = True

    return closest_point, status

def transform_point(goal_point):
    goal_pose_msg = PoseStamped()
    goal_pose_msg.header.stamp = rospy.Time.now()
    goal_pose_msg.header.frame_id = "map"
    goal_pose_msg.pose.position.x = goal_point[0]
    goal_pose_msg.pose.position.y = goal_point[1]
    goal_pose_msg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0.0, 0.0, goal_point[2])
    goal_pose_msg.pose.orientation.x = quaternion[0]
    goal_pose_msg.pose.orientation.y = quaternion[1]
    goal_pose_msg.pose.orientation.z = quaternion[2]
    goal_pose_msg.pose.orientation.w = quaternion[3]

    tf_listener.waitForTransform("/map", "/base_link", goal_pose_msg.header.stamp, rospy.Duration(0.5))
    goal_pose = tf_listener.transformPose("/base_link", goal_pose_msg)
    # (trans,rot) = listener.lookupTransform('/map/', '/base_link/', rospy.Time(0))

    return goal_pose_msg, goal_pose


def callback(msg):

    x = msg.pose.pose.position.x  
    y = msg.pose.pose.position.y

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    pose_quaternion = np.array([qx, qy, qz, qw])
    yaw = tf.transformations.euler_from_quaternion(pose_quaternion)[2]

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    found = False
    global previous_goal
    goal_point = previous_goal
    while( not found):
        goal_point, status = find_goal_point(x,y)
        if status:
            found =True


    # 3. Transform the goal point to vehicle coordinates. 
    goal_pose_msg, goal_pose = transform_point(goal_point)

    global LOOKAHEAD_DISTANCE
    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.
    abs_y = abs(goal_pose.pose.position.y)
    angle = ((2.0*abs_y)/(LOOKAHEAD_DISTANCE**2))

    if (goal_pose.pose.position.y < 0):
       angle = -angle #Right Steering
    else:
        angle = angle #Left Steering

    
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
    # clipping  speeds and Lookahead
    degree_angle =  math.degrees(angle)
    if abs(degree_angle) < ANGLE_LEVEL_1:
        vel =  SPEED_LEVEL_1
        LOOKAHEAD_DISTANCE = 2
    elif  ANGLE_LEVEL_1<= abs(degree_angle) and abs(degree_angle) < ANGLE_LEVEL_2:
        vel =  SPEED_LEVEL_2
        LOOKAHEAD_DISTANCE = 1.5
    else:
        vel =  SPEED_LEVEL_3
        LOOKAHEAD_DISTANCE = 1.0


    msg = drive_param()
    msg.velocity = vel
    msg.angle = angle
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.Subscriber('pf/pose/odom', Odometry, callback, queue_size=1)
    rospy.spin()

