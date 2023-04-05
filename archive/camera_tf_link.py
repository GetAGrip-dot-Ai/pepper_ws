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



def listener():
    rospy.init_node('tf_node_2')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # rospy.Subscriber("/tf", TFMessage, tf_callback)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('camera_bottom_screw_frame',
                                              'base_link',
                                              rospy.Time.now(),
                                              rospy.Duration(0.001))
            print("trans:", trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
            rate.sleep()
            continue


if __name__ == '__main__':
    listener()

