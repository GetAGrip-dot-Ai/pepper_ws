#!/usr/bin/env python3
import rospy
from realsense_camera import RealsenseCamera
from realsense_utils import *


def check_center_depth():
    rs_camera = RealsenseCamera()

    visualize_color_point_cloud(rs_camera)


if __name__ == "__main__":
    check_center_depth()