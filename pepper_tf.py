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
import numpy as np
from std_msgs.msg import Float32, Int64MultiArray

def getDepthCallback(msg):
    global depth_img
    depth_img = np.reshape(msg, (480,640))

def tfCallback(msg):
    br = tf.TransformBroadcaster()
    
    for t in msg.transforms:
        # if (t.header.frame_id=="bracelet_link"):
        if (t.child_frame_id=="rs_ee"):
            # print(t.header.frame_id) # spherical_wrist_2_link
            pepper_tf = geometry_msgs.msg.TransformStamped()
            pepper_tf.transform.translation.x = 2
            pepper_tf.transform.translation.y = 2
            pepper_tf.transform.translation.z = depth_img[pepper_tf.transform.translation.x][pepper_tf.transform.translation.y]
            pepper_tf.transform.rotation = t.transform.rotation

            # rot = t.transform.rotation
            # rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            # r = rpy[0] + math.pi
            # p = rpy[1] + math.pi/4 + math.pi/2
            # # p = rpy[1] + math.pi/2
            # y = rpy[2] 
            # quat = quaternion_from_euler(r, p, y)
            # quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
            # pepper_tf.transform.rotation = quat_msg
            pepper_tf.header.stamp = rospy.Time.now()
            # camera_tf.header.frame_id = "bracelet_link"
            # camera_tf.child_frame_id = "camera_frame"
            # print(camera_tf.transform.translation.x)
            # br.sendTransform(camera_tf)
            br.sendTransform((pepper_tf.transform.translation.x, pepper_tf.transform.translation.y, pepper_tf.transform.translation.z),(pepper_tf.transform.rotation.x, pepper_tf.transform.rotation.y, pepper_tf.transform.rotation.z, pepper_tf.transform.rotation.w), time=pepper_tf.header.stamp , child = "pepper", parent="rs_ee")

def tf_listener():
    rospy.init_node('pepper_tf_node')
    rospy.Subscriber("/tf", TFMessage, tfCallback)
    rospy.Subscriber("/camera/pp/depth", Int64MultiArray, getDepthCallback)
    rospy.spin()

if __name__ == '__main__':
    tf_listener()