#!/usr/bin/env python

import rospy
from race.msg import drive_param
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool

pub = rospy.Publisher('carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size=1)
vehicle_control_manual_override_publisher = rospy.Publisher("/carla/ego_vehicle/vehicle_control_manual_override", Bool, queue_size=1, latch=True)
auto_pilot_enable_publisher = rospy.Publisher("/carla/ego_vehicle/enable_autopilot", Bool, queue_size=1)
vehicle_control_manual_override = True
autopilot_enabled = False

def control_callback(msg):   
    data = CarlaEgoVehicleControl()
    data.throttle = msg.velocity
    data.steer = msg.angle
    data.brake = 0
    data.gear = 1
    data.reverse = False
    data.manual_gear_shift = True
    pub.publish(data)
    vehicle_control_manual_override_publisher.publish(vehicle_control_manual_override)
    auto_pilot_enable_publisher.publish(autopilot_enabled)

if __name__ == '__main__':
    rospy.init_node('carla_sim_connector', anonymous=True)
    rospy.Subscriber("drive_parameters", drive_param, control_callback)
    rospy.spin()
