#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from tf2_msgs.msg import TFMessage
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


def tf_callback(msg):
    print("==========================")
    for t in msg.transforms:
        print(t.header.frame_id)
        
        if (t.header.frame_id=="camera_depth_optical_frame"): # should be the world frame
            print(" this is a thing ")
        if (t.header.frame_id=="camera_depth_frame"): # should be the world frame
            print(" this is a thing 11")

def listener():
    rospy.init_node('tf_node')
    rospy.Subscriber("/tf_static", TFMessage, tf_callback)
    '''
    base_link
    shoulder_link
    half_arm_1_link
    half_arm_2_link
    forearm_link
    spherical_wrist_1_link
    spherical_wrist_2_link

    ==========================
    end_effector_link
    end_effector_link
    end_effector_link
    bracelet_link
    end_effector_link
    world
    ==========================
    realsense_frame
    camera_bottom_screw_frame
    ==========================
    camera_link
    camera_depth_frame
    this is a thing 11
    camera_link
    camera_color_frame
    camera_link
    camera_infra1_frame
    camera_link
    camera_infra2_frame
    camera_link
    camera_gyro_frame
    camera_link
    camera_accel_frame
    '''

    rospy.spin()

if __name__ == '__main__':
    listener()

