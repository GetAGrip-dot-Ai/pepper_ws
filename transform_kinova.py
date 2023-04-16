#!/usr/bin/env python3  
import rospy
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':
    rospy.init_node('camera_calibration')

    listener = tf.TransformListener()
    
    rospy.sleep(10)

    now = rospy.Time.now()
    listener.waitForTransform("/base_link", "/aruco_marker", now, rospy.Duration(10.0))
    (p_aruco_base, q_aruco_base) = listener.lookupTransform("/base_link", "/aruco_marker", now)

    R_aruco_base = R.from_quat([q_aruco_base[0], q_aruco_base[1], q_aruco_base[2], q_aruco_base[3]])

    print(f"p_aruco_base {p_aruco_base}")
    print(f"q_aruco_base {q_aruco_base}")

    with open("/home/sridevi/kinova_ws/src/pepper_ws/aruco_wrt_base_kinova_camera.npy", "wb") as f:
        np.save(f, np.array(p_aruco_base))
        np.save(f, np.array(q_aruco_base))
