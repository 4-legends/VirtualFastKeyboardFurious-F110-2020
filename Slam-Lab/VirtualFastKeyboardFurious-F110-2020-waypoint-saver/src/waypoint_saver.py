#!/usr/bin/env python

import rospy
import numpy as np
import tf

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

csv_file = open('../waypoints.csv', 'w')

pub = rospy.Publisher('/saved_waypoints', Path, queue_size=1)
waypoints = Path()

def saveCallback(msg):

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    pose_quaternion = np.array([qx, qy, qz, qw])
    theta = tf.transformations.euler_from_quaternion(pose_quaternion)[2]
    print (x,y, theta)
    csv_file.write('{}, {}, {}\n'.format(x,y, theta))

    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose

    waypoints.header = msg.header
    waypoints.poses.append(pose_stamped)

    pub.publish(waypoints)


if __name__ == '__main__':
    rospy.init_node('waypoints_saver', anonymous=True)
    rospy.Subscriber('pf/pose/odom', Odometry, saveCallback)
    rospy.spin()
