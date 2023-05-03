import os
import time

import matplotlib.pyplot as plt
import rospy
import time
from termcolor import colored


from one_frame import OneFrame
from multi_frame import MultiFrame
from realsense_camera import RealsenseCamera
from communication import Communication
from pepper_fruit_utils import *
from pepper_utils import *
from realsense_utils import *
from multi_frame_utils import *     
from pepper_peduncle_detector import PepperPeduncleDetector


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: The main pipeline that is used by the multi-frame and visual servoing service
"""


class Perception:
    def __init__(self, multi_frame_number = 10, threshold=0.5, percentage=0.5):
        # self.source = source
        self.rs_camera = RealsenseCamera()
        self.threshold = threshold
        self.percentage = percentage
        self.pepper_fruits = dict()
        self.pepper_peduncles = dict()
        self.peppers = dict()
        # self.one_frame = None # TODO
        self.multi_frame = MultiFrame(multi_frame_number)
        self.communication = Communication()
        self.poi_in_rviz = None # TODO
        self.chosen_pepper = None


    def set_pepper_order(self, arm_xyz):
        #################################################################
        # Determine the order in which peppers must be picked
        # output:
        #   self.pepper.order must be set
        #################################################################
        self.peppers = determine_pepper_order(self.peppers.values(), arm_xyz)
    

    def add_frame_to_multi_frame(self):
        #################################################################
        # Take an image and add it as a frame to the multi frame
        #################################################################

        rgb_img = get_image(self.rs_camera)

        if rgb_img is None:
            print(colored("NO IMAGE READ BY THE RGBD CAMERA", "red"))

        number = 0 if not self.multi_frame._one_frames else len(self.multi_frame._one_frames)

        print(colored(f"Frame #{number} added to multiframe", "blue"))

        img_path = os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png'
        cv2.imwrite(img_path, rgb_img) 
        # TODO: change the way we save images 

        self.multi_frame.add_one_frame(OneFrame(img_path, number))
        self.multi_frame.populate_last_frame(self.rs_camera)

    def process_multi_frame(self):
        #################################################################
        # Get the point of interaction from the multi frame
        #################################################################
        self.multi_frame.run()

        matched_positive_peppers = self.multi_frame._matched_positive_peppers

        print(colored(f"Matched Positive Peppers: {matched_positive_peppers}", 'yellow'))

        if not matched_positive_peppers:
            plot_frames(self.multi_frame, -1, -1)
            print(colored("No peppers here!", "red"))
            return
        else:
            for frame_number, peppers in matched_positive_peppers.items():

                for pepper in peppers:

                    self.chosen_pepper = pepper

                    print(colored(f"Chosen frame: {frame_number}, Chosen pepper: {self.chosen_pepper.pepper_fruit.number}", "blue"))

                    plot_frames(self.multi_frame, frame_number, self.chosen_pepper.pepper_fruit.number)

                    try: 
                        if frame_number in self.multi_frame._unmatched_positive_fruits.keys():
                            self.pepper_fruits = self.multi_frame._matched_positive_fruits[frame_number] + self.multi_frame._unmatched_positive_fruits[frame_number]
                        else:
                            self.pepper_fruits = self.multi_frame._matched_positive_fruits[frame_number]

                        self.pepper_fruits.remove(self.chosen_pepper.pepper_fruit)
                        print(colored(f"Pepper fruits after deleting chosen pepper: {self.pepper_fruits}", "blue"))

                    except Exception as e:
                        print(colored(f"Key error while deleting pepper fruit from obstacles: {e}", "red"))

                    return

        # self.set_pepper_order(arm_xyz) # TODO

    def send_to_manipulator(self):

        #################################################################
        # send the point of interaction to the manipulator over ROS
        #################################################################


        if self.chosen_pepper is None:
            self.multi_frame.clear_frames()
            self.multi_frame = MultiFrame()
            return 0
        
        else:
            poi_in_base_link = self.chosen_pepper.pepper_peduncle.poi_in_base_link

            print(colored(f"Chosen POI:\n{poi_in_base_link}", "green"))

            rate = rospy.Rate(10)

            start_time = time.time()

            while not rospy.is_shutdown() and time.time()- start_time < 1:
                
                self.communication.publish_poi(poi_in_base_link, None)
                # self.communication.obstacle_pub_fn(self.pepper_fruits)
                # self.communication.rviz_marker_poi_base_link(self.peppers)
                # self.communication.rviz_marker(poi_in_base_link)
                # self.communication.rviz_marker_rs(poi_in_base_link)
                # self.communication.publish_poi([poi[0], poi[1], poi[2]], None)

                # rate.sleep()

            self.multi_frame.clear_frames()
            self.multi_frame = MultiFrame()
            self.chosen_pepper = None

            return 1