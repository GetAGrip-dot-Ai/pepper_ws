#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from tf2_msgs.msg import TFMessage
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


def tf_callback(msg):
    # print("===========")
    # print(msg)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.transforms)
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    
    for t in msg.transforms:
        # if (t.header.frame_id=="bracelet_link"):
        if (t.child_frame_id=="bracelet_link"):
            # print(t.header.frame_id) # spherical_wrist_2_link
            camera_tf = geometry_msgs.msg.TransformStamped()
            camera_tf.transform.translation.x = t.transform.translation.x
            camera_tf.transform.translation.y = t.transform.translation.y
            camera_tf.transform.translation.z = t.transform.translation.z
            # camera_tf.transform.rotation = t.transform.rotation

            rot = t.transform.rotation
            rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            r = rpy[0] - math.pi/2
            p = rpy[1]
            y = rpy[2] + math.pi/2
            quat = quaternion_from_euler(r, p, y)
            quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
            camera_tf.transform.rotation = quat_msg
            camera_tf.header.stamp = rospy.Time.now()
            # camera_tf.header.frame_id = "bracelet_link"
            # camera_tf.child_frame_id = "camera_frame"
            # print(camera_tf.transform.translation.x)
            # br.sendTransform(camera_tf)
            br1.sendTransform((camera_tf.transform.translation.x, camera_tf.transform.translation.y, camera_tf.transform.translation.z),(camera_tf.transform.rotation.x, camera_tf.transform.rotation.y, camera_tf.transform.rotation.z, camera_tf.transform.rotation.w), time=camera_tf.header.stamp , child = "rs_ee", parent="bracelet_link")


        if (t.header.frame_id=="base_link"): # should be the world frame
            # print(t.header.frame_id)
            camera_tf_base = geometry_msgs.msg.TransformStamped()
            camera_tf_base.transform.translation.x = 0.01
            camera_tf_base.transform.translation.y = 0
            camera_tf_base.transform.translation.z = 0
            # camera_tf_base.transform.rotation = t.transform.rotation

            rot = t.transform.rotation
            rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            r = 0
            p = 0
            y = 0
            quat = quaternion_from_euler(r, p, y)
            quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
            camera_tf_base.transform.rotation = quat_msg
            camera_tf_base.header.stamp = rospy.Time.now()
            # camera_tf.header.frame_id = "bracelet_link"
            # camera_tf.child_frame_id = "camera_frame"
            # print(camera_tf.transform.translation.x)
            # print(camera_tf)
            # br.sendTransform(camera_tf)
            br2.sendTransform((camera_tf_base.transform.translation.x, camera_tf_base.transform.translation.y, camera_tf_base.transform.translation.z),(camera_tf_base.transform.rotation.x, camera_tf_base.transform.rotation.y, camera_tf_base.transform.rotation.z, camera_tf_base.transform.rotation.w), time=camera_tf_base.header.stamp , child = "rs_base", parent="world")

def listener():
    rospy.init_node('tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

