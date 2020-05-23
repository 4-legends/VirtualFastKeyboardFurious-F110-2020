#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import copy

import math
import numpy as np
import tf 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import time
from lane_finding import LaneFinder
import message_filters
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError



MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
LOOKAHEAD_DISTANCE = 1.5  
DESIRED_DISTANCE = 0.7  


class LaneCentering:
    def __init__(self): 
 
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.des_pub = rospy.Publisher('desired_point', PoseStamped, queue_size=1)
        self.cur_pub = rospy.Publisher('current_point', PoseStamped, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.lane_pub = rospy.Publisher('lane_images', Image, queue_size=1)


        self.KP = 0.6
        self.KD = 0.01

        self.past_error = 0.0
        self.flag = False
        self.bridge = CvBridge()
        self.laneFind = LaneFinder()

        self.image_sub = message_filters.Subscriber('/carla/ego_vehicle/camera/rgb/front/image_color', Image)
        self.depth_sub = message_filters.Subscriber('/carla/ego_vehicle/camera/depth/view/image_depth', Image)
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10) 
        # ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.dataCallback)


    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 +  (p1[2] - p2[2]) ** 2)

    def update_vel(self, angle):
        ang_deg =  math.degrees(angle)
        if abs(ang_deg) >= 0.0 and abs(ang_deg) <= 10.0:
            vel = 1.5

        elif abs(ang_deg) > 10.0 and abs(ang_deg) <= 20.0:
            vel = 1.0

        else:
            vel = 0.5
        return vel


    def compute_error(self, ref_points):
        first_right, second_right, center = ref_points
        a = self.dist(second_right, center)
        b = self.dist(first_right, center)
        THETA  = np.arccos(b/a)
        alpha = 0.0
        num = a*np.cos(np.radians(THETA)) - b
        denom= a*np.sin(np.radians(THETA))
        alpha = np.arctan2(num, denom)

        Dt = b*np.cos(alpha)
        Dt_projected = Dt + LOOKAHEAD_DISTANCE*np.sin(alpha)
        right_error =  DESIRED_DISTANCE - Dt_projected
        return right_error

    def dataCallback(self, rbg_msg, depth_msg):

        try:           
            rgb_image = self.bridge.imgmsg_to_cv2(rbg_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        try:
          depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except CvBridgeError as e:
          print(e)
        #print(type(depth_image))
        print ('Subscribing to depth and RGB Image Stream')
        lane_imgs, ref_points, flag = self.laneFind.processor(rgb_image, depth_image)
        
        if flag:
            lane_msg = self.bridge.cv2_to_imgmsg(lane_imgs, encoding="bgr8")
            error = self.compute_error(ref_points)
            self.PID_contoller(error)
            self.lane_bkp = lane_imgs
        else:
            lane_msg = self.bridge.cv2_to_imgmsg(self.lane_bkp, encoding="bgr8")
        
        self.lane_pub.publish(lane_msg)
 

    def PID_contoller(self, curr_error):

        delta_time = 0.025
        delta_error = curr_error - self.past_error

        self.past_error = curr_error

        delta_error_delta_time = delta_error/delta_time

        if self.flag:
            pidoutput = self.KP*curr_error + self.KD*delta_error_delta_time
        else:
            self.flag = True
            pidoutput = self.KP*curr_error

         
        angle = np.clip(pidoutput, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
        velocity = self.update_vel(angle)
        
        msg = drive_param()
        msg.velocity = velocity
        msg.angle = angle
        self.drive_pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('lane_centering')
    pp_obj = LaneCentering()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Lane Centering...")

