#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *

pipeline = Perception(None, 0)

def handle_multi_frame(req):
    if req.req_id == 0:
        pipeline.add_frame_to_multi_frame()
        return 1
    else:
        pipeline.process_multi_frame()
        poi_detected = pipeline.send_to_manipulator()
        return poi_detected

def multi_frame_server(mult):
    rospy.init_node('multi_frame_server')
    s = rospy.Service('/perception/multi_frame', multi_frame, handle_multi_frame)
    rospy.spin()

if __name__ == "__main__":
    multi_frame_server()