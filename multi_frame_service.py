#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *
from termcolor import colored
import rospkg


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 12, 2023
Code description: Server in the harvest service which performs the multi-frame process
"""


rospack = rospkg.RosPack()

perception = None

def handle_multi_frame(req):
    global perception

    if req == 0:
        print(colored("Manipulation system requested to start multiframe", "blue"))
        perception.add_frame_to_multi_frame()
        return 1
    else:
        print(colored("Manipulation system requested to process multiframe", "blue"))
        perception.add_frame_to_multi_frame() # TODO fix sri's code
        perception.process_multi_frame()
        pepper_found = perception.send_to_manipulator()
        return pepper_found
    
def multi_frame_server():
    global perception

    rospy.init_node('multi_frame_server')
    rate = rospy.Rate(10) # TODO: Change this to 10
    os.chdir(rospack.get_path("pepper_ws"))

    perception = Perception()

    handle_multi_frame(0)
    handle_multi_frame(0)
    handle_multi_frame(0)
    handle_multi_frame(1)
    s = rospy.Service('/perception/harvest', multi_frame, handle_multi_frame)
    rospy.spin()

if __name__ == "__main__":
    print(colored("In Service Main", "blue"))
    multi_frame_server()