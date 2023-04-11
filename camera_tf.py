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
    br3 = tf.TransformBroadcaster()
    
    for t in msg.transforms:
        
        if (t.child_frame_id=="bracelet_link"):
            # making rs_ee tf frame
            # set translations
            camera_tf = geometry_msgs.msg.TransformStamped()
            camera_tf.transform.translation.x = 0
            camera_tf.transform.translation.y = 0.08
            camera_tf.transform.translation.z = -0.2
            # rotate frame
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
            # broadcast tf
            camera_tf.transform.rotation = quat_msg
            camera_tf.header.stamp = rospy.Time.now()
            br1.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "rs_ee", parent="bracelet_link")
        

            # making realsense_frame
            # set translations
            rgbd_tf = geometry_msgs.msg.TransformStamped()
            rgbd_tf.transform.translation.x = 0
            rgbd_tf.transform.translation.y = 0
            rgbd_tf.transform.translation.z = 0
            # rotate frame
            quat3 = quaternion_from_euler(math.pi, 0, 0)
            quat_msg = Quaternion(quat3[0], quat3[1], quat3[2], quat3[3])
            rgbd_tf.transform.rotation = quat_msg
            # broadcast tf
            rgbd_tf.header.stamp = rospy.Time.now()
            br2.sendTransform((rgbd_tf.transform.translation.x, rgbd_tf.transform.translation.y, rgbd_tf.transform.translation.z),(rgbd_tf.transform.rotation.x, rgbd_tf.transform.rotation.y, rgbd_tf.transform.rotation.z, rgbd_tf.transform.rotation.w), time=rgbd_tf.header.stamp , child = "realsense_frame", parent="rs_ee")

            # making ee frame
            # translate
            ee_tf = geometry_msgs.msg.TransformStamped()
            ee_tf.transform.translation.x = 0
            ee_tf.transform.translation.y = 0
            ee_tf.transform.translation.z = 0
            # rotate
            quat3 = quaternion_from_euler(math.pi, 0, 0)
            quat_msg = Quaternion(quat3[0], quat3[1], quat3[2], quat3[3])
            rgbd_tf.transform.rotation = quat_msg
            # broadcast
            rgbd_tf.header.stamp = rospy.Time.now()
            br3.sendTransform((rgbd_tf.transform.translation.x, rgbd_tf.transform.translation.y, rgbd_tf.transform.translation.z),(rgbd_tf.transform.rotation.x, rgbd_tf.transform.rotation.y, rgbd_tf.transform.rotation.z, rgbd_tf.transform.rotation.w), time=rgbd_tf.header.stamp , child = "realsense_frame", parent="rs_ee")

def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

