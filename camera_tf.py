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
from std_msgs.msg import Float32, Int64MultiArray
from tf.transformations import *

def tf_callback(msg):
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    
    for t in msg.transforms:
        if (t.child_frame_id=="bracelet_link"):
            
            camera_tf = geometry_msgs.msg.TransformStamped()
            # camera_tf.transform.translation.x = t.transform.translation.x +1
            # camera_tf.transform.translation.y = t.transform.translation.y
            # camera_tf.transform.translation.z = t.transform.translation.z

            camera_tf.transform.translation.x = 0
            camera_tf.transform.translation.y = -0.2
            camera_tf.transform.translation.z = 0

            rot = t.transform.rotation
            # rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            r = 0
            p = math.pi/2
            y = 0
            quat_1 = quaternion_from_euler(r, p, y)

            r = math.pi/2
            p = 0
            y = 0
            quat_2 = quaternion_from_euler(r, p, y)

            quat = quaternion_multiply( quat_1, quat_2)
            # quat_msg = Quaternion(0, 0, 0, 1)
            quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
            camera_tf.transform.rotation = quat_msg
            camera_tf.header.stamp = rospy.Time.now()
            br1.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "rs_ee", parent="bracelet_link")

        # not using base camera for SVD
        # if (t.header.frame_id=="base_link"): # should be the world frame
        #     camera_tf_base = geometry_msgs.msg.TransformStamped()
        #     camera_tf_base.transform.translation.x = 0.01
        #     camera_tf_base.transform.translation.y = 0
        #     camera_tf_base.transform.translation.z = 0

        #     rot = t.transform.rotation
        #     rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        #     r = 0
        #     p = 0
        #     y = 0
        #     quat = quaternion_from_euler(r, p, y)
        #     quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
        #     camera_tf_base.transform.rotation = quat_msg
        #     camera_tf_base.header.stamp = rospy.Time.now()
        #     br2.sendTransform((camera_tf_base.transform.translation.x, camera_tf_base.transform.translation.y, camera_tf_base.transform.translation.z),(camera_tf_base.transform.rotation.x, camera_tf_base.transform.rotation.y, camera_tf_base.transform.rotation.z, camera_tf_base.transform.rotation.w), time=camera_tf_base.header.stamp , child = "rs_base", parent="world")
        
def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

