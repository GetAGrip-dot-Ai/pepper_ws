#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from one_frame import OneFrame
from realsense_utils import *
from termcolor import colored
import rospkg
from realsense_camera import RealsenseCamera
from realsense_utils import *
import numpy as np
from pepper_utils import draw_all_multi_frame

"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 12, 2023
Code description: Server in the harvest service which performs the multi-frame process
"""

class TestMessage:
    def __init__(self, req_id) -> None:
        self.req_id = req_id

rospack = rospkg.RosPack()

img_path = 'testimg.jpeg'

def handle_one_frame(req, rs_camera):
    poi = np.array([0, 0, 0])
    if req.req_id == 0:
        print(colored("Manipulation system requested to start oneframe", "blue"))
        cv2.imwrite(img_path, get_image(rs_camera))
        test_frame = OneFrame(img_path, 0)
        test_frame.run(rs_camera)
        for _, single_pepper in test_frame._pepper_detections.items():
            print("poi is: ", single_pepper.pepper_peduncle.poi)
            poi = single_pepper.pepper_peduncle.poi
    draw_all_multi_frame(test_frame)
    visualize_color_point_cloud(rs_camera, poi)
    print("node done")
    
def one_frame_server(rs_camera):
    rospy.init_node('one_frame_server')
    rate = rospy.Rate(10) # TODO: Change this to 10
    os.chdir(rospack.get_path("pepper_ws"))

    test_0 = TestMessage(0)
    handle_one_frame(test_0, rs_camera)

if __name__ == "__main__":
    rs_camera = RealsenseCamera()
    print(colored("In Service Main", "blue"))
    one_frame_server(rs_camera)