#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import os
import csv
from virtualfastkeyboardfurious_wall_following.msg import gaps
import tf


rospy.init_node('instructionFollowing', anonymous=True) 
tf_listener = tf.TransformListener()

# SET GAP FINDING CONSTANTS
TURN_COMPLETE_ANGLE = 50
CENTER_COUNTER = 400
R_GAP_ANGLE_LIMITS= rospy.get_param('R_GAP_ANGLE_LIMITS')
L_GAP_ANGLE_LIMITS = rospy.get_param('L_GAP_ANGLE_LIMITS')
MIN_GAP_SIZE= rospy.get_param('MIN_GAP_SIZE')
MAX_GAP_SIZE = rospy.get_param('MAX_GAP_SIZE')

class instructionFollowing():
    def __init__(self, instruction_file):
        self.read_instruction_file(instruction_file)
        self.instruction_index = -1
        print ('\n\nLOADED EXPLICT INSTRUCTIONS: FOLLOWING DEFAULT INSTRUCTION: Lets Begin')
        self.print_instruction_status()
        self.instruction_index = 0

        self.default_direction = rospy.get_param('DIRECTION')
        self.default_velocity = rospy.get_param('VELOCITY')
        self.center_no_junction_counts = 0    
        self.heading_at_start = 0.0

        self.gap_sub = rospy.Subscriber("lidar_gaps", gaps, self.process_callback)

    def read_instruction_file(self, instruction_file):
        with open(instruction_file) as f:
            instructions = [tuple(line) for line in csv.reader(f)]
        self.instructions = [(point[0], float(point[1])) for point in instructions]

    def print_instruction_status(self):
        print ('\n*********************\n')

        for i in range(len(self.instructions)):
            if i==self.instruction_index:
                print (self.instructions[i], ' <-- INSTRUCTION TO BE EXCECUTED')
            else:
                print (self.instructions[i])
        print ('\n*********************\n')

    def active_turn_monitoring(self, junction_type):
        direction = rospy.get_param('DIRECTION')
        delta_heading = abs(self.find_robot_heading() - self.heading_at_start)
        
        if direction == 'left' or direction == 'right':
            if  delta_heading >= TURN_COMPLETE_ANGLE:
                print('FOLLOWING DEFAULT INSTRUCTION: TURN COMPLETE')
                rospy.set_param('INSTRUCTION_STATUS', 'INACTIVE')
                    
        if direction == 'center':
            if junction_type == 'NO_JUNCTION':
                self.center_no_junction_counts += 1

            if self.center_no_junction_counts >= CENTER_COUNTER:
                print('FOLLOWING DEFAULT INSTRUCTION: TURN COMPLETE')
                rospy.set_param('INSTRUCTION_STATUS', 'INACTIVE')
                self.center_no_junction_counts = 0

        if direction == 'stop':
            self.default_direction = 'stop'
            self.default_velocity = 0.0
            rospy.set_param('INSTRUCTION_STATUS', 'ACTIVE')            
            rospy.set_param('DIRECTION', 'stop')
            rospy.set_param('VELOCITY', 0.00)

    def find_robot_heading(self):
        _, quaternion = tf_listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        roll_pitch_yaw = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(roll_pitch_yaw[2])
        return yaw 


    def get_junction_type(self, gap_msg):
        gap_angle_list = gap_msg.gap_angle
        gap_size_list = gap_msg.gap_size
        gap_at_left = False
        gap_at_right = False
        gap_at_cneter = False
        junction_type = 'NO_JUNCTION'

        for i in range(len(gap_size_list)):
            cur_gap_size = gap_size_list[i]
            cur_gap_angle = gap_angle_list[i]
            if MIN_GAP_SIZE <= cur_gap_size  and cur_gap_size <= MAX_GAP_SIZE:
                if cur_gap_angle <= R_GAP_ANGLE_LIMITS:
                    gap_at_right = True
                elif cur_gap_angle > R_GAP_ANGLE_LIMITS and cur_gap_angle < L_GAP_ANGLE_LIMITS:
                    gap_at_cneter = True
                elif cur_gap_angle >= L_GAP_ANGLE_LIMITS:
                    gap_at_left = True

        if gap_at_left == True and gap_at_right == True and gap_at_cneter == True:
            junction_type = 'CROSS'
        elif gap_at_left == True and gap_at_right == True: # T Junction Case
            junction_type = 'T'
        elif gap_at_left == True and gap_at_cneter == True:  # T Junction Case
            junction_type = 'T'
        elif gap_at_cneter == True and gap_at_right == True: # T Junction Case
            junction_type = 'T'
        return junction_type

    def handle_junction(self, junction_type):
        self.print_instruction_status()
        self.yaw_start = self.find_robot_heading()
        direction, vel = self.instructions[self.instruction_index]
        rospy.set_param('INSTRUCTION_STATUS', 'ACTIVE')
        rospy.set_param('DIRECTION', direction)
        rospy.set_param('VELOCITY', vel)
        self.instruction_index +=1        


    def process_callback(self, gap_msg):
        junction_type  = self.get_junction_type(gap_msg)
        if rospy.get_param('INSTRUCTION_STATUS') == 'INACTIVE':
            if junction_type == 'CROSS' or junction_type == 'T':
                print('FOLLOWING EXPLICIT INSTRUCTION: TURN INITIATED')
                self.handle_junction(junction_type)

            elif junction_type == 'NO_JUNCTION':
                rospy.set_param('DIRECTION', self.default_direction)
                rospy.set_param('VELOCITY', self.default_velocity)

        if rospy.get_param('INSTRUCTION_STATUS') == 'ACTIVE':
            self.active_turn_monitoring(junction_type)

if __name__ == '__main__':
    instruction_file  = rospy.get_param('INSTRUCTIONS_FILE')
    instructionFollowing(instruction_file)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    