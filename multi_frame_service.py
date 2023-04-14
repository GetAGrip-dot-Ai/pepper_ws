#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *

global pipeline

def handle_multi_frame(req):
    print(f"Request ID: {req.req_id}")
    if req.req_id == 0:
        print("In 0")
        pipeline.add_frame_to_multi_frame()
        return 1
    elif req.req_id == 1:
        print("In 1")
        frame_number = pipeline.process_multi_frame()
        return frame_number
    else:
        print("In 2")
        pipeline.send_to_manipulator()
        return 1
    
def multi_frame_server():
    global pipeline
    print("1")
    rospy.init_node('multi_frame_server')
    rate = rospy.Rate(1)
    os.chdir('/home/sridevi/kinova_ws/src/pepper_ws/')
    print("2")
    pipeline = Perception(None, 0)
    print("3")
    s = rospy.Service('/perception/multi_frame', multi_frame, handle_multi_frame)
    print("4")
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == "__main__":
    print("In Service Main")
    multi_frame_server()