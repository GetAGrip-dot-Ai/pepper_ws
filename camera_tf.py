#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from tf2_msgs.msg import TFMessage
import tf
import math
from tf.transformations import *

def tf_callback(msg):
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    
    for t in msg.transforms:
        if (t.child_frame_id=="bracelet_link"):
            
            camera_tf = geometry_msgs.msg.TransformStamped()
            camera_tf.transform.translation.x = 0
            camera_tf.transform.translation.y = -0.2
            camera_tf.transform.translation.z = 0

            r = 0
            p = math.pi/2
            y = 0
            quat_1 = quaternion_from_euler(r, p, y)

            r =  math.pi/2
            p = 0
            y = 0
            quat_2 = quaternion_from_euler(r, p, y)
            quat = quaternion_multiply( quat_1, quat_2)
            quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])

            camera_tf.transform.rotation = quat_msg
            camera_tf.header.stamp = rospy.Time.now()
            br1.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "rs_ee", parent="bracelet_link")
        
            rgbd_tf = geometry_msgs.msg.TransformStamped()
            rgbd_tf.transform.translation.x = 0
            rgbd_tf.transform.translation.y = 0
            rgbd_tf.transform.translation.z = 0

            quat3 = quaternion_from_euler(0, 0, 0)

            quat_msg = Quaternion(quat3[0], quat3[1], quat3[2], quat3[3])
            
            rgbd_tf.transform.rotation = quat_msg
            rgbd_tf.header.stamp = rospy.Time.now()

            br2.sendTransform((rgbd_tf.transform.translation.x, rgbd_tf.transform.translation.y, rgbd_tf.transform.translation.z),(rgbd_tf.transform.rotation.x, rgbd_tf.transform.rotation.y, rgbd_tf.transform.rotation.z, rgbd_tf.transform.rotation.w), time=rgbd_tf.header.stamp , child = "realsense_frame", parent="rs_ee")

def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

