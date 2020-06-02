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
from visualization_msgs.msg import Marker

#############
# CONSTANTS #
#############
global LOOKAHEAD_DISTANCE
LOOKAHEAD_DISTANCE = 1.5 # meters

global previous_goal
previous_goal = 0

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
filename = os.path.join(dirname, '../waypoints/pure-pursuit-waypoints.csv')

with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
goal_pub = rospy.Publisher('desired_path', PoseStamped, queue_size=1)
marker_pub = rospy.Publisher('lookahead',Marker, queue_size=1)

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


def find_goal_point(x,y, yaw, LOOKAHEAD_DISTANCE):

    dist_arr= np.zeros(len(path_points))
    global previous_goal
    goal = previous_goal

    for i in range(len(path_points)):
        dist_arr[i] = dist((path_points[i][0],path_points[i][1]),(x,y))

    goal_arr = []
    for i in range(dist_arr.shape[0]):
        if dist_arr[i]>= LOOKAHEAD_DISTANCE- 0.3 and dist_arr[i] <= LOOKAHEAD_DISTANCE +0.3:
            goal_arr.append(i)

    for idx in goal_arr:
        vector1 = [path_points[idx][0]-x , path_points[idx][1]-y]
        vector2 = [np.cos(yaw), np.sin(yaw)]
        if abs(find_angle(vector1, vector2)) < np.pi/2:
            goal = idx
            break

    return goal

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

def find_angle( vector1, vector2):
    cos_comp = np.dot(vector1, vector2)
    sin_comp = np.linalg.norm(np.cross(vector2, vector2))
    return np.arctan2(sin_comp, cos_comp)

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
    global LOOKAHEAD_DISTANCE

    goal = find_goal_point(x,y, yaw, LOOKAHEAD_DISTANCE)

    goal_point = path_points[goal]
    previous_goal = goal


    # # 3. Transform the goal point to vehicle coordinates. 
    goal_pose_msg, goal_pose = transform_point(goal_point)

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.
    abs_y = abs(goal_pose.pose.position.y)
    curvature = ((2.0*abs_y)/(LOOKAHEAD_DISTANCE**2))
    angle = np.arctan(LOOKAHEAD_DISTANCE*curvature)
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
    goal_pub.publish(goal_pose_msg)


    lookahead_marker = Marker()
    lookahead_marker.type = Marker.TEXT_VIEW_FACING
    lookahead_marker.header.frame_id = '/map'
    lookahead_marker.scale.z = 0.5
    lookahead_marker.color.a = 1
    lookahead_marker.color.r = 1.0
    lookahead_marker.color.g = 0.0
    lookahead_marker.color.b = 0.0    
    lookahead_marker.pose.position.x = 0.0
    lookahead_marker.pose.position.y = 4.0
    rospy.set_param('LOOKAHEAD_DISTANCE', LOOKAHEAD_DISTANCE) 
    multiline_str = 'LOOKAHEAD_DISTANCE: %s'%str(LOOKAHEAD_DISTANCE) + ' \n' + 'VELOCITY: %s'%str(vel)

    lookahead_marker.text =  multiline_str 
    marker_pub.publish(lookahead_marker)

if __name__ == '__main__':
    rospy.Subscriber('/pf/pose/odom', Odometry, callback, queue_size=1)
    rospy.spin()

