#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_callback(data):
    Maxlook = 5
    ranges = np.asarray(data.ranges)
    max1 = 0
    Angle = 10
    Minangle = 0.25
    for i in range(int(Angle/Minangle)):
        if ranges[i+(int((135- Angle)/Minangle))] > max1:
            max1 = ranges[i+(int((135- Angle)/Minangle))]
    if max1 > Maxlook:
        vel = 10
    else:
        vel = 2

    rospy.set_param('Velocity', vel)



 # Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('lookahead_update', anonymous=True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
