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

depth_img = np.ones((480, 640)) * -1
x, y = -1, -1 
def getDepthCallback(msg):
    global depth_img
    depth_img = np.reshape(msg.data, (480,640))
    # print(depth_img[0, 0])

def xy_callback(msg):
    global x, y
    if x < 0:
        # print(msg.data)
        x, y = np.array(msg.data)

def tfCallback(msg):
    br = tf.TransformBroadcaster()
    
    # for t in msg.transforms:
    if depth_img[0,0] != -1:
        # x, y = 240, 320
        pepper_tf = geometry_msgs.msg.TransformStamped()
        pepper_tf.transform.translation.z = (x-320)/1000
        pepper_tf.transform.translation.y = (y-240)/1000
        pepper_tf.transform.translation.x = depth_img[x][y] /1000

        # rot = t.transform.rotation
        # rpy = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        # r = rpy[0] + math.pi
        # p = rpy[1] + math.pi/4 + math.pi/2
        # # p = rpy[1] + math.pi/2
        # y = rpy[2] 
        quat = quaternion_from_euler(0, 0, 0)
        quat_msg = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pepper_tf.transform.rotation = quat_msg
        pepper_tf.header.stamp = rospy.Time.now()
        # camera_tf.header.frame_id = "bracelet_link"
        # camera_tf.child_frame_id = "camera_frame"
        # print(camera_tf.transform.translation.x)
        # br.sendTransform(camera_tf)
        br.sendTransform((pepper_tf.transform.translation.x, pepper_tf.transform.translation.y, pepper_tf.transform.translation.z),(pepper_tf.transform.rotation.x, pepper_tf.transform.rotation.y, pepper_tf.transform.rotation.z, pepper_tf.transform.rotation.w), time=pepper_tf.header.stamp , child = "pepper", parent="rs_ee")
    else:
        print("waiting for image to be a thing")
def tf_listener():
    rospy.init_node('pp_p_pepper_tf_node')
    rospy.Subscriber("/camera/pp/depth", Int64MultiArray, getDepthCallback)
    rospy.Subscriber("/tf", TFMessage, tfCallback)
    rospy.Subscriber("/pp/poi_test", Int64MultiArray, xy_callback)
    rospy.spin()

if __name__ == '__main__':
    tf_listener()