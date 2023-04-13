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
    
    for t in msg.transforms:
        
        if (t.child_frame_id=="bracelet_link"):
            # set translations
            camera_tf = geometry_msgs.msg.TransformStamped()
            camera_tf.transform.translation.x = 0
            camera_tf.transform.translation.y = -0.21
            camera_tf.transform.translation.z = -0.132
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
            br1.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "realsense_frame", parent="bracelet_link")
    

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
    br4.sendTransform((msg.position.x + 0.0325, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="base_link")
    print("###############")


def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/perception/peduncle/poi", Pose, pepper_tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

