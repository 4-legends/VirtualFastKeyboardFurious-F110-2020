#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Vector3
from virtualfastkeyboardfurious_wall_following.msg import gaps

#this publishes the drive command to run the car.
drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=5)

#this publishes the center of the gap in(x,y,z) coordinate system
gap_center_pub = rospy.Publisher("gap_center", Vector3, queue_size=5)

#Implement your own message to publish relevant information about gaps
gaps_pub = rospy.Publisher("lidar_gaps", gaps, queue_size=5)

max_gap_distance = 10.0
min_gap_distance = 0.5

global prev_gap_msg_info
prev_gap_msg_info = gaps()

def find_gaps(data):

    angle_increment = data.angle_increment # angle increment over scan steps
    min_angle = data.angle_min #min angle of laser scanner

    data_arr = np.array(data.ranges)

    data_arr[np.isnan(data_arr)] = 0.0
    #filter data
    data_arr[data_arr>max_gap_distance] = max_gap_distance
    data_arr[data_arr<min_gap_distance] = min_gap_distance

    segment_storage = []
    scale_permissible_diff = 0.1
    #finding gaps based on sudden difference between consecutive range values
    for i in range(0, data_arr.shape[0]-1):
        delta = abs(data_arr[i+1] - data_arr[i])
        if delta>scale_permissible_diff*data_arr[i]:
            segment_storage.append(i)

    gap = Vector3()
    gap_length_msg = Float64()
    gap_angle_msg = Float64()

    gap_msg_info = gaps()

    for i in range(len(segment_storage)-1):
        #angle between gap points
        scan_gap_angle = angle_increment*(segment_storage[i+1]-segment_storage[i])

        #two sides of the gap 
        s1 = data_arr[segment_storage[i]] 
        s2 = data_arr[segment_storage[i]]

        gap_length = np.sqrt( s1**2 + s2**2 - 2*s2*s1*np.cos(scan_gap_angle))
        alpha =np.arccos(1/(2*s1*gap_length)*(s1**2 + gap_length*gap_length - s2**2))

        #vector from robot position to center of gap
        r_vector = np.sqrt(s1**2 + (gap_length/2)**2 - s1*gap_length*np.cos(alpha))
        center_angle = np.arccos((1/(2*s1*r_vector))*(s1**2 + r_vector**2 -(gap_length/2)**2 ))

        #transforming the angle of robot origin to scan reference frame.
        start_angle = angle_increment*segment_storage[i]
        projection_angle = start_angle + center_angle + min_angle
  
        #cartesian to polar coordinates
        gap.x = np.cos(projection_angle)*r_vector
        gap.y = np.sin(projection_angle)*r_vector
        gap.z = 0.0
        gap_msg_info.gap_centers.append(gap)

        gap_length_msg.data = gap_length
        gap_msg_info.gap_size.append(gap_length_msg)
        
        gap_angle = np.degrees(projection_angle)

        gap_angle_msg.data = gap_angle
        gap_msg_info.gap_angle.append(gap_angle_msg)

        gap_msg_info.num_gaps +=1

    # if gap_msg_info.num_gaps<1:
    #     global prev_gap_msg_info

    #     gap_msg_info = prev_gap_msg_info

    #     best_gap_idx = np.argmax(gap_msg_info.gap_size)
    #     best_gap_vector = gap_msg_info.gap_centers[best_gap_idx]   
    # else:
    best_gap_idx = np.argmax(gap_msg_info.gap_size)
    best_gap_vector = gap_msg_info.gap_centers[best_gap_idx]   
    return gap_msg_info, best_gap_vector

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):

    gap_msg_info, best_gap_vector = find_gaps(data)


    #publishing gap center and gap_info messages
    gap_center_pub.publish(best_gap_vector)
    gaps_pub.publish(gap_msg_info)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('gap_finding_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()

