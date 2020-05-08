#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
LOOKAHEAD_DISTANCE = 1.5  
DESIRED_DISTANCE = 0.7  
#uncomment below for center following
# LOOKAHEAD_DISTANCE =  2.5
THETA = 45
# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):


  data_arr = np.array(data.ranges)
  #filter data
  data_arr[data_arr>MAX_DISTANCE] = MAX_DISTANCE
  data_arr[data_arr<MIN_DISTANCE] = MIN_DISTANCE

  # check if within fixed angle bounds 
  if angle<MIN_ANGLE:
    angle = MIN_ANGLE
  elif angle>MAX_ANGLE:
    angle = MAX_ANGLE

  # find Index of range value
  ranges_idx = int((angle - MIN_ANGLE)/np.degrees(data.angle_increment))
  range_val = data_arr[ranges_idx]

  return range_val


def compute_projectedDistance(a, b):
    num = a*np.cos(np.radians(THETA)) - b
    denom= a*np.sin(np.radians(THETA))
    alpha = np.arctan2(num, denom)

    Dt = b*np.cos(alpha)
    Dt_projected = Dt + LOOKAHEAD_DISTANCE*np.sin(alpha)
    return Dt_projected

# data: single message from topic /scanc
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):

  #diametrically opposite to right side.
  b = getRange(data, 180.0)
  a = getRange(data, 180. - THETA)

  Dt_projected =compute_projectedDistance(a,b)
  left_error =  Dt_projected - desired_distance

  return left_error

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):  
  a = getRange(data, THETA)
  b = getRange(data, 0.0)

  Dt_projected = compute_projectedDistance(a,b)
  right_error =  desired_distance - Dt_projected

  return right_error

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data, DESIRED_DISTANCE):
  left_error = followLeft(data, DESIRED_DISTANCE)
  right_error = followRight(data, DESIRED_DISTANCE)
  return (left_error + right_error)/2.0 #  

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  # error = followRight(data, DESIRED_DISTANCE)
  error = followLeft(data, DESIRED_DISTANCE)
  # error = followCenter(data, DESIRED_DISTANCE)

  string = 'left'
  rospy.loginfo('%s error %f', string, error)

  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.spin()
