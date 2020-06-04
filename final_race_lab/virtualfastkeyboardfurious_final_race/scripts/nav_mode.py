#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

key_pub = rospy.Publisher('key', String, queue_size=1)
def reset_callback(reset_msg):
    if reset_msg.data == True:
        rospy.sleep(0.5)
        key = String()
        key.data = 'n'
        key_pub.publish(key)


if __name__ == '__main__':
    rospy.init_node('Key_Node', anonymous=True)
    reset_sub = rospy.Subscriber('/reset_car', Bool, reset_callback, queue_size=1 )
    rospy.sleep(1)
    key = String()
    key.data = 'n'
    key_pub.publish(key)
    rospy.spin()
