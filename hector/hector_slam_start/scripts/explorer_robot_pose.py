#!/usr/bin/env python


import rospy
import roslib
from tf.transformations import *
from tf import *
from serial import *
from io import *

def send_robot_pose_transform(serial_data):
    br = TransformBroadcaster()
    current_roll = 0.0
    current_pitch = 0.0
    current_yaw = 0.0
    prase_serial_data = serial_data.split(' ')
    current_roll = float(prase_serial_data[0]) * 0.0175
    current_pitch = float(prase_serial_data[1]) * 0.0175
    br.sendTransform((0,0,0),quaternion_from_euler(current_roll,current_pitch,current_yaw),rospy.Time.now(),'base_link','base_stabilized')

if __name__=='__main__':
    rospy.init_node('explorer_robot_pose')
    rate = rospy.Rate(100)
    print('[INFO] Transform has begined!')
    while rospy.is_shutdown() is not True:
        line = "0 0"
        send_robot_pose_transform(line)
        rate.sleep()

