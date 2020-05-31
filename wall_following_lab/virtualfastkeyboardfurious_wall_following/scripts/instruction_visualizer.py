#!/usr/bin/env python

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import rospy

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
text_marker_pub = rospy.Publisher('instruction_status_text', Marker, queue_size=1)

# Input data is Vector3 representing center of largest gap
def callback(data):
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.header.frame_id = '/map'
    text_marker.scale.z = 0.5
    text_marker.color.a = 1.0
    text_marker.color.b = 0.0    
    text_marker.pose.position.x = 0.0
    text_marker.pose.position.y = 4.0


    direction = rospy.get_param('DIRECTION')
    vel = rospy.get_param('VELOCITY')
    INSTRUCTION_STATUS = rospy.get_param('INSTRUCTION_STATUS')

    if INSTRUCTION_STATUS == 'ACTIVE':
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        multiline_str = 'INSTRUCTION STATUS: %s'%INSTRUCTION_STATUS + ' \n' +  'DIRECTION: %s '%direction  + ' \n' 'VELOCITY: %s'%str(vel)


    elif INSTRUCTION_STATUS == 'INACTIVE':
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        multiline_str = 'DEFAULT MODE'  + ' \n' +  'DEFAULT DIRECTION: %s '%direction  + ' \n' 'DEFAULT VELOCITY: %s'%str(vel)

    text_marker.text =  multiline_str 
    text_marker_pub.publish(text_marker)

if __name__ == '__main__':
    rospy.init_node('instruction_visualizer')
    rospy.Subscriber('/odom', Odometry, callback, queue_size=1)
    rospy.spin()
