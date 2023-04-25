#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *
from termcolor import colored

import rospkg

rospack = rospkg.RosPack()


perception = None
first_request = True

def handle_multi_frame(req):
    global first_request
    global perception
    # print(colored(f"Request ID: {req.req_id}", "blue"))

    if req.req_id == 0:
        if not first_request:
            perception.rs_camera.pipeline.start(perception.rs_camera.config)
            time.sleep(3)
            print(colored("Camera reinitialized", "light_green"))
        else:
            print(colored("Camera not need to initialize", "light_green"))
            first_request = False
        print(colored("Manipulation system requested to start multiframe", "blue"))
        perception.add_frame_to_multi_frame()
        return 1
    else:
        print(colored("Manipulation system requested to process multiframe", "blue"))
        perception.add_frame_to_multi_frame()
        perception.process_multi_frame()
        pepper_found = perception.send_to_manipulator()
        perception.rs_camera._pipeline.stop()
        return pepper_found
    
def multi_frame_server():
    global perception

    rospy.init_node('multi_frame_server')
    rate = rospy.Rate(1) # TODO: Change this to 
    os.chdir(rospack.get_path("pepper_ws"))

    perception = Perception()

    # handle_multi_frame(0)
    # handle_multi_frame(0)
    # handle_multi_frame(0)
    # handle_multi_frame(1)
    s = rospy.Service('/perception/harvest', multi_frame, handle_multi_frame)
    rospy.spin()

if __name__ == "__main__":
    print(colored("In Service Main", "blue"))
    multi_frame_server()