#!/usr/bin/env python3  
import rospy
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Code that calculates the transform of the realsense camera w.r.t the bracelet link of the kinova arm
"""


if __name__ == '__main__':
    rospy.init_node('camera_calibration')

    listener = tf.TransformListener()
    
    rospy.sleep(10)

    with open("/home/sridevi/kinova_ws/src/pepper_ws/aruco_wrt_base_kinova_camera.npy", "rb") as f:
        p_aruco_base = np.load(f)
        q_aruco_base = np.load(f)

    # print(p_aruco_base)
    # print(q_aruco_base)

    R_aruco_base = R.from_quat([q_aruco_base[0], q_aruco_base[1], q_aruco_base[2], q_aruco_base[3]]).as_matrix()

    H_aruco_base = np.vstack((np.hstack((R_aruco_base, p_aruco_base.reshape(3, 1))), np.array([0, 0, 0, 1])))

    now = rospy.Time.now()
    listener.waitForTransform("/realsense_frame", "/aruco_marker", now, rospy.Duration(10.0))
    (p_aruco_realsense, q_aruco_realsense) = listener.lookupTransform("/realsense_frame", "/aruco_marker", now)

    R_aruco_realsense = R.from_quat([q_aruco_realsense[0], q_aruco_realsense[1], q_aruco_realsense[2], q_aruco_realsense[3]]).as_matrix()

    H_aruco_realsense = np.vstack((np.hstack((R_aruco_realsense, np.array(p_aruco_realsense).reshape(3, 1))), np.array([0, 0, 0, 1])))

    H_realsense_base = np.matmul(H_aruco_base, np.linalg.inv(H_aruco_realsense))

    now = rospy.Time.now()
    listener.waitForTransform("/base_link", "/bracelet_link", now, rospy.Duration(10.0))
    (p_bracelet_base, q_bracelet_base) = listener.lookupTransform("/base_link", "/bracelet_link", now)

    R_bracelet_base = R.from_quat([q_bracelet_base[0], q_bracelet_base[1], q_bracelet_base[2], q_bracelet_base[3]]).as_matrix()

    H_bracelet_base = np.vstack((np.hstack((R_bracelet_base, np.array(p_bracelet_base).reshape(3, 1))), np.array([0, 0, 0, 1])))

    H_realsense_bracelet = np.matmul(np.linalg.inv(H_bracelet_base), H_realsense_base)

    print(f"H_realsense_bracelet {H_realsense_bracelet}")

    with open("/home/sridevi/kinova_ws/src/pepper_ws/realsense_wrt_bracelet.npy", "wb") as f:
        np.save(f, np.array(H_realsense_bracelet))

    with open("/home/sridevi/kinova_ws/src/pepper_ws/realsense_wrt_bracelet.npy", "rb") as f:
        print(f"Loaded value {np.load(f)}")
    print("rewrf")
    # with open("/home/sridevi/kinova_ws/src/pepper_ws/realsense_wrt_bracelet2.npy", "rb") as f:
    #     print(f"Loaded value {np.load(f)}")
       
