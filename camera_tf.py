#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from tf2_msgs.msg import TFMessage
import tf
import math
from tf.transformations import *
import numpy as np
from scipy.spatial.transform import Rotation as R

def tf_callback(msg):
    br1 = tf.TransformBroadcaster()
    
    with open("/home/sridevi/kinova_ws/src/pepper_ws/realsense_wrt_bracelet.npy", "rb") as f:
        H_realsense_bracelet = np.load(f)

    R_realsense_bracelet = np.asmatrix(H_realsense_bracelet[:3, :3])
    H = R.from_matrix(R_realsense_bracelet)
    q_realsense_bracelet = H.as_quat()

    # rotate frame
    r = math.pi/2
    p = 0
    y = 0
    quat_1 = quaternion_from_euler(r, p, y)
    q_realsense_bracelet = quaternion_multiply(q_realsense_bracelet, quat_1)

    r = 0
    p = 0
    y = math.pi/2
    quat_2 = quaternion_from_euler(r, p, y)
    q_realsense_bracelet = quaternion_multiply( q_realsense_bracelet, quat_2)

    
    p_realsense_bracelet = H_realsense_bracelet[:3, 3]
    
    br1.sendTransform((p_realsense_bracelet[0], p_realsense_bracelet[1], p_realsense_bracelet[2]), (q_realsense_bracelet[0], q_realsense_bracelet[1], q_realsense_bracelet[2], q_realsense_bracelet[3]), time=rospy.Time.now(), child = "realsense_frame", parent="bracelet_link")

    

def pepper_tf_callback(msg):
    print("pepper_tf_callback")
    point = msg.position

    if not rospy.has_param('pepper_tf'):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)
    
    elif round(float(rospy.get_param('pepper_tf').split(',')[0]), 3) != round(float(point.x), 3):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)

    br4 = tf.TransformBroadcaster()
    br4.sendTransform((msg.position.x, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="base_link")
    print("###############")


def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/perception/peduncle/poi", Pose, pepper_tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

