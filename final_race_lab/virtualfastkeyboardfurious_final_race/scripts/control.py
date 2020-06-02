#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math
import os

# TODO: modify these constants to make the car follow walls smoothly.
KP = rospy.get_param('KP')
KD = rospy.get_param('KD')
DIRECTION = rospy.get_param('DIRECTION')


ANGLE_LEVEL_1 = rospy.get_param('ANGLE_LEVEL_1')
SPEED_LEVEL_1 = rospy.get_param('SPEED_LEVEL_1')
ANGLE_LEVEL_2 = rospy.get_param('ANGLE_LEVEL_2')
SPEED_LEVEL_2 = rospy.get_param('SPEED_LEVEL_2')
SPEED_LEVEL_3 =rospy.get_param('SPEED_LEVEL_3')


past_error = 0.0
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):
    curr_error = msg.data

    global past_error

    pid_output = KP*curr_error + KD*(curr_error - past_error)/0.025  
    past_error = curr_error

    angle = pid_output
    angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    # Car starts driving down center
    direction = rospy.get_param('DIRECTION')
    vel = rospy.get_param('VELOCITY')

    if DIRECTION == 'stop':
        vel = 0.0
        angle = 0.0

    degree_angle =  math.degrees(angle)
    if  0.0 <= abs(degree_angle)  and abs(degree_angle) <= ANGLE_LEVEL_1:
        vel = vel
    elif  ANGLE_LEVEL_1<= abs(degree_angle) and abs(degree_angle) <= ANGLE_LEVEL_2:
        vel = SPEED_LEVEL_2
    else:
        vel = SPEED_LEVEL_3

    rospy.loginfo('vel %f angle_degrees %f angle_rad %f curr_error %f DIRECTION %s', vel, degree_angle, angle, curr_error, direction)
    msg = drive_param()
    msg.velocity = vel   
    msg.angle = angle  
    pub.publish(msg)


# Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("pid_error", Float64, control_callback)
    rospy.spin()
