#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
import tf
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf.transformations import *
import rospkg


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 4, 2023
Code description: Reads the stored transform of the realsense frame w.r.t the bracelet link of the kinova arm, rotates it and publishes it
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


'''

rospack = rospkg.RosPack()

def tf_callback(msg):
    br1 = tf.TransformBroadcaster()
    
    H_realsense_bracelet = np.array(
        [
            [0.63651,   -0.770701 , -0.029576 , 0.0363789],
            [0.770983  ,  0.636849, -0.00277229 , -0.0222353],
            [0.0209721, -0.021038  ,  0.999559 , 0.169965],
            [  0    ,       0     ,      0        ,   1]
        ]
    )

    R_realsense_bracelet = np.asmatrix(H_realsense_bracelet[:3, :3])
    H = R.from_matrix(R_realsense_bracelet)
    q_realsense_bracelet = H.as_quat()

    # rotate frame
    # r = math.pi/2
    # p = 0
    # y = 0
    # quat_1 = quaternion_from_euler(r, p, y)
    # q_realsense_bracelet = quaternion_multiply(q_realsense_bracelet, quat_1)

    # r = 0
    # p = 0
    # y = math.pi/2
    # quat_2 = quaternion_from_euler(r, p, y)
    # q_realsense_bracelet = quaternion_multiply( q_realsense_bracelet, quat_2)

    
    p_realsense_bracelet = H_realsense_bracelet[:3, 3]
    
    br1.sendTransform((p_realsense_bracelet[0], p_realsense_bracelet[1], p_realsense_bracelet[2]), (q_realsense_bracelet[0], q_realsense_bracelet[1], q_realsense_bracelet[2], q_realsense_bracelet[3]), time=rospy.Time.now(), child = "realsense_frame", parent="link_eef")


def pepper_tf_callback(msg):
    point = msg.position

    if not rospy.has_param('pepper_tf'):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)
    
    elif round(float(rospy.get_param('pepper_tf').split(',')[0]), 3) != round(float(point.x), 3):
        poi_str = f"{point.x},{point.y},{point.z}"
        rospy.set_param('pepper_tf', poi_str)

    br4 = tf.TransformBroadcaster()
    br4.sendTransform((msg.position.x + 0.0325, msg.position.y, msg.position.z),(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), time=rospy.Time.now(), child = "pepper_tf", parent="base_link")


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
