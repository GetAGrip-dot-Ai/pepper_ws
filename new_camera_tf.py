#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
import tf
import math
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf.transformations import *
import rospkg


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 4, 2023
Code description: Reads the stored transform of the realsense frame w.r.t the bracelet link of the xarm, rotates it and publishes it
"""

'''
    0.63651   -0.770701   -0.029576   0.0363789
   0.770983    0.636849 -0.00277229  -0.0222353
  0.0209721   -0.021038    0.999559    0.169965
          0           0           0           1

0.726162   -0.687412  -0.0123754  0.00890136
   0.687455    0.726226 -0.00097093  -0.0328987
 0.00965478 -0.00780249    0.999923  -0.0234608
          0           0           0           1

   0.722161   -0.691161  -0.0279334   0.0129646
   0.691719    0.721391    0.033468  -0.0672875
-0.00298082  -0.0434913    0.999049   0.0075407
          0           0           0           1

0.70246   -0.710825   0.0357387 -0.00429454
0.71006    0.703365   0.0330363  -0.0256654
-0.0486204  0.00216994    0.998815     0.13882
          0           0           0           1



'''

rospack = rospkg.RosPack()

def tf_callback(msg):
    # br1 = tf.TransformBroadcaster()
    
    # TODO: save in a log file
    H_realsense_bracelet = np.array(
        [
            [0.70246,   -0.710825 , 0.0357387 , -0.00429454],
            [0.71006  ,  0.703365, 0.0330363 , -0.0256654],
            [-0.0486204, 0.00216994  ,  0.998815 , 0.13882],
            [  0    ,       0     ,      0        ,   1]
        ]
    )

    R_realsense_bracelet = np.asmatrix(H_realsense_bracelet[:3, :3])
    H = R.from_matrix(R_realsense_bracelet)
    q_realsense_bracelet = H.as_quat()

    p_realsense_bracelet = H_realsense_bracelet[:3, 3]
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "link_eef"
    static_transformStamped.child_frame_id = "realsense_frame"

    static_transformStamped.transform.translation.x = float(p_realsense_bracelet[0])
    static_transformStamped.transform.translation.y = float(p_realsense_bracelet[1])
    static_transformStamped.transform.translation.z = float(p_realsense_bracelet[2])

    quat = (q_realsense_bracelet[0], q_realsense_bracelet[1], q_realsense_bracelet[2], q_realsense_bracelet[3])

    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)
    # br1.sendTransform((p_realsense_bracelet[0], p_realsense_bracelet[1], p_realsense_bracelet[2]), (q_realsense_bracelet[0], q_realsense_bracelet[1], q_realsense_bracelet[2], q_realsense_bracelet[3]), time=rospy.Time.now(), child = "realsense_frame", parent="link_eef")


def pepper_tf_callback(msg):
    point = msg.position

    if not rospy.has_param('pepper_tf'):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)
    
    elif round(float(rospy.get_param('pepper_tf').split(',')[0]), 3) != round(float(point.x), 3):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)

    br4 = tf.TransformBroadcaster()
    # br4.sendTransform((msg.position.x + 0.0325, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="link_base")
    br4.sendTransform((msg.position.x, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="link_base")


def listener():
    rospy.init_node('camera_tf_node')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/perception/peduncle/poi", Pose, pepper_tf_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
