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

class ImageSub:
    def __init__(self): 
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/carla/ego_vehicle/camera/depth/view/image_depth', Image, self.callback)


        self.image_sub = message_filters.Subscriber('/carla/ego_vehicle/camera/rgb/front/image_color', Image)
        self.depth_sub = message_filters.Subscriber('/carla/ego_vehicle/camera/depth/view/image_depth', Image)
        ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10) 
        # ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.dataCallback)


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
        print ('heloo')
        cv2.imwrite('I1mageWindows.jpg', rgb_image)

        cv2.imwrite('ImageWindows.jpg', depth_image)
        #cv2.waitKey(0) 

if __name__ == '__main__':
    rospy.init_node('image_sub')
    pp_obj = ImageSub()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down Lane Centering...")

