#!/usr/bin/env python3
import rospy
from realsense_camera import RealsenseCamera
from realsense_utils import *


def check_center_depth():
    rs_camera = RealsenseCamera()

    while True:
        _, _, depth = get_depth(rs_camera)
        print(f"Depth in inches = {39.3701*depth}")


if __name__ == "__main__":
    check_center_depth()