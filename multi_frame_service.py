#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *
from termcolor import colored

global pipeline

def handle_multi_frame(req):
    print(colored(f"Request ID: {req.req_id}"))
    if req.req_id == 0:
        print("In 0")
        pipeline.add_frame_to_multi_frame()
        return 1
    else:
        print("In 1")
        pipeline.process_multi_frame()
        pepper_found = pipeline.send_to_manipulator()
        return pepper_found
    
def multi_frame_server():
    global pipeline
    rospy.init_node('multi_frame_server')
    rate = rospy.Rate(1)
    os.chdir('/home/sridevi/kinova_ws/src/pepper_ws/')
    pipeline = Perception(None, 0)
    s = rospy.Service('/perception/multi_frame', multi_frame, handle_multi_frame)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == "__main__":
    print("In Service Main")
    multi_frame_server()