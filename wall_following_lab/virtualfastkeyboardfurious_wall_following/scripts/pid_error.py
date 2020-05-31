#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = rospy.get_param('MIN_DISTANCE')
MAX_DISTANCE = rospy.get_param('MAX_DISTANCE')
MIN_ANGLE = rospy.get_param('MIN_ANGLE')
MAX_ANGLE = rospy.get_param('MAX_ANGLE')


THETA = rospy.get_param('THETA')
LOOKAHEAD_DISTANCE = rospy.get_param('LOOKAHEAD_DISTANCE')
DESIRED_DISTANCE = rospy.get_param('DESIRED_DISTANCE')

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


def compute_projectedDistance(a, b, flag=''):
    num = a*np.cos(np.radians(THETA)) - b
    denom= a*np.sin(np.radians(THETA))
    alpha = np.arctan2(num, denom)

    Dt = b*np.cos(alpha)
    if flag =='center':
        Dt = 0
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

  return left_error, Dt_projected

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):  
  a = getRange(data, THETA)
  b = getRange(data, 0.0)

  Dt_projected = compute_projectedDistance(a,b)
  right_error =  desired_distance - Dt_projected

  return right_error, Dt_projected

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data, DESIRED_DISTANCE):
    a = getRange(data, THETA)
    b = getRange(data, 0)
    theta = np.deg2rad(THETA)
    tan_alpha = (a*math.cos(theta) - b)/(a*math.sin(theta))
    alpha = math.atan(tan_alpha)

    Dt_projected = compute_projectedDistance(a,b, 'center')

    _, left_dist  = followLeft(data, DESIRED_DISTANCE)
    _, right_dist  = followRight(data, DESIRED_DISTANCE)
    center_error = left_dist - right_dist - Dt_projected  
    return center_error

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
    left_error, _ = followLeft(data, DESIRED_DISTANCE)
    right_error, _ = followRight(data, DESIRED_DISTANCE)
    center_error = followCenter(data, DESIRED_DISTANCE)

    DIRECTION = rospy.get_param('DIRECTION')
    
    if DIRECTION == 'left':
        error = left_error
    elif DIRECTION == 'right':
       error = right_error
    elif DIRECTION == 'center':
       error = center_error
    elif DIRECTION == 'stop':
        error = 0.0
        print("Stop Car!!")


    msg = Float64()
    msg.data = error
    pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    rospy.init_node('pid_error_node', anonymous = True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
