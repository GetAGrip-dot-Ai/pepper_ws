#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
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

            # making fake bracelet frame bc end-effector is mounted rotated 90 clockwise
            # set translations
            real_bracelet_tf = geometry_msgs.msg.TransformStamped()
            real_bracelet_tf.transform.translation.x = 0 #t.transform.translation.x
            real_bracelet_tf.transform.translation.y = 0 #t.transform.translation.y
            real_bracelet_tf.transform.translation.z = 0 #t.transform.translation.z
            # rotate frame
            quat3 = quaternion_from_euler(0, 0, math.pi/2)
            quat_msg = Quaternion(quat3[0], quat3[1], quat3[2], quat3[3])
            real_bracelet_tf.transform.rotation = quat_msg
            # broadcast tf
            real_bracelet_tf.header.stamp = rospy.Time.now()
            br1.sendTransform((real_bracelet_tf.transform.translation.x, real_bracelet_tf.transform.translation.y, real_bracelet_tf.transform.translation.z),(real_bracelet_tf.transform.rotation.x, real_bracelet_tf.transform.rotation.y, real_bracelet_tf.transform.rotation.z, real_bracelet_tf.transform.rotation.w), time=real_bracelet_tf.header.stamp, child = "real_bracelet_link", parent="bracelet_link")

        if (t.child_frame_id=="real_bracelet_link"):

            # making rs_ee tf frame
            # set translations
            camera_tf = geometry_msgs.msg.TransformStamped()
            camera_tf.transform.translation.x = 0 #-0.02
            camera_tf.transform.translation.y = 0.03 #0.04 # real: -0.021  #+z
            camera_tf.transform.translation.z = -0.17 #-0.092  #-x
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
            br2.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "rs_ee", parent="real_bracelet_link")
        

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
            br3.sendTransform((rgbd_tf.transform.translation.x, rgbd_tf.transform.translation.y, rgbd_tf.transform.translation.z),(rgbd_tf.transform.rotation.x, rgbd_tf.transform.rotation.y, rgbd_tf.transform.rotation.z, rgbd_tf.transform.rotation.w), time=rgbd_tf.header.stamp , child = "realsense_frame", parent="rs_ee")

def pepper_tf_callback(msg):
    br4 = tf.TransformBroadcaster()
    br4.sendTransform((msg.position.x, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="base_link")

def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/perception/peduncle/poi", Pose, pepper_tf_callback)
    rospy.spin()

    # bracelet to tool (probs right)
    # [0.0, 0.0, -0.0615250000000001] trans
    # [1.0, 8.880412265862542e-48, -5.4968537584186e-33, 1.6155445744325867e-15]


    # tool to bracelet (probs wrong)
    # [-6.763878549734098e-34, 1.9879275988393013e-16, -0.0615250000000001]


if __name__ == '__main__':
    listener()

