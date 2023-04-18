# sys.path.insert(0, '/home/shri/Perception_Resources/yolov8_scripts/src')
import os
import time

import matplotlib.pyplot as plt
import rospy
import time
from termcolor import colored

print(os.getcwd())

from one_frame import OneFrame
from multi_frame import MultiFrame
from communication import Communication
from pepper_fruit_utils import *
from pepper_utils import *
from realsense_utils import *
from multi_frame_utils import *     


# input: image
class Perception:
    def __init__(self, source, fps, multi_frame_number = 10, threshold=0.5, percentage=0.5, save=True):
        self.source = source
        self.start_time = time.time()
        self.fps = fps
        self.save = save
        self.threshold = threshold
        self.percentage = percentage
        self.pepper_fruits = dict()
        self.pepper_peduncles = dict()
        self.peppers = dict()
        self.one_frame = None
        self.multi_frame = MultiFrame(multi_frame_number)
        self.communication = Communication()
        self.poi_in_rviz = None
        self.chosen_pepper = None

    def get_image(self):
        #################################################################
        # get image from source
        # output: RGBD information
        #################################################################
        if self.source == "webcam":
            self.image = get_image_from_webcam()
        else:
            self.image = read_image(self.source)

    def get_depth(self, image, x, y):
        #################################################################
        # given an image and x, y coordinates, return the depth information
        # note this will take in an area in the image, average the depth
        # and do additional calculation to compensate for the noise given
        # by the RGBD camera
        # input:
        #   image: (H, W, D)?
        #   x, y: coordinates
        # output:
        #   d: depth of that point
        #################################################################
        pass

    #####################################################################
    # When base goes to one location, use the long range images and retreive
    # all the locations of peppers
    #####################################################################
    def detect_peppers_one_frame(self, path, thresh=0.5):
        #################################################################
        # use yolov8_scripts and get the pepper locations
        # input:
        #   path: image path
        #   thresh: disgard detection lower than threshold
        # output:
        #   locations: all the locations of the pepper boxes [conf, x, y] (N, 3)
        #################################################################
        self.one_frame = OneFrame(path)
        self.one_frame.run()
        self.pepper_fruits = self.one_frame.pepper_fruit_detections
        self.pepper_peduncles = self.one_frame.pepper_peduncle_detections
        print(f"Pepper peduncles: {self.pepper_peduncles}")
        self.peppers = self.one_frame.pepper_detections
        return self.pepper_peduncles[0].poi

    def detect_peppers_in_folder(self):
        files = get_all_image_path_in_folder(self.source)
        print(os.getcwd(),"==", self.source)
        for path in files:
            self.detect_peppers_one_frame(path)
            
    def detect_peppers_realtime(self):
        complete = False
        while not complete:
            user_input = input("1: start\n2: end\n")
            if user_input == "1":
                print("taking pic!")
                img = get_image()
                img_name=str(time.time()).split('.')[0]
                cv2.imwrite(os.getcwd()+'/realtime/'+img_name+'.png', img)
                print("saved to :", os.getcwd()+'/realtime/'+img_name+'.png')
                try:
                    (poi_x, poi_y, poi_z) = self.detect_peppers_one_frame(os.getcwd()+'/realtime/'+img_name+'.png')
                    self.poi_in_rviz = (poi_x, poi_y, poi_z)
                    complete = True
                    return (poi_x, poi_y, poi_z)
                except Exception as e:
                    print("no pepper detected")
                    print(e)
            elif user_input == "2":
                return False
            else:
                print("invalid option!")

    def detect_peppers_time_frame(self, frames, thresh=0.5):
        #################################################################
        # JIYOON TODO
        # stack pepper locations over a timeframe time
        # input:
        #   frames: F number of frames to be stored in list
        # output:
        #   locations: all the locations of the pepper boxes over a
        #       number of frames F x [conf, x, y] (F, N, 3)
        #################################################################
        for i in range(frames):
            pepper_detection = self.detect_peppers_one_frame(i, thresh)
            self.pepper_fruit[i] = pepper_detection  # store dictionary of pepper in frame number
        # print(self.pepper_fruit)

    def clear_false_positives(self):
        #################################################################
        # TODO
        # algorithm to take in a series of bounding boxes and output
        # true positive pepper locations
        # input:
        #   locations : pepper locations over F frames (F, N, 3)
        # output:
        #   self.pepper_locations: true positive pepper locations including
        #   the depth information (M, 4)
        #################################################################
        pass

    #####################################################################
    # Once the manipulator goes closer to the pepper, we have one pepper
    # as target.
    #####################################################################
    def set_target_pepper(self, pepper_index):
        #################################################################
        # Using the pepper_index, take another closer image of the pepper,
        # run the detection algorithm to get a more precise bounding box.
        # Store the pepper's information in self.pepper
        # output:
        #   self.pepper = {"idx": None, "box": (L, T, R, D), "location": (xc, yc, d)}
        #################################################################
        pass

    def set_pepper_order(self, arm_xyz):
        #################################################################
        # Determine the order in which peppers must be picked
        # output:
        #   self.pepper.order must be set
        #################################################################
        self.peppers = determine_pepper_order(self.peppers.values(), arm_xyz)
    
    #####################################################################
    # ROS related
    #####################################################################

    def add_frame_to_multi_frame(self):
        #################################################################
        # Take an image and add it as a frame to the multi frame
        #################################################################
        img = get_image()
        if img == None:
            print(colored("NO IMAGE READ BY THE RGBD CAMERA", "red"))

        number = 0 if not self.multi_frame._one_frames else len(self.multi_frame._one_frames)
        print(colored(f"Frame number being added to multi-frame: {number}", "blue"))
        # print("-------", os.getcwd())
        cv2.imwrite(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png', img)
        self.multi_frame.add_one_frame(OneFrame(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png'))
        self.multi_frame.assign_last_frame_number(number)
        self.multi_frame.populate_last_frame()

    def process_multi_frame(self):
        #################################################################
        # Get the point of interaction from the multi frame
        #################################################################
        self.multi_frame.run()
        peppers_temp = self.multi_frame._matched_positive_peppers
        print(colored(f"Peppers: {peppers_temp}", 'red'))

        print(f"Matched positive peppers keys: {peppers_temp.keys()}")
        print(f"Matched positive fruits keys: {self.multi_frame._matched_positive_fruits.keys()}")
        print(f"Unmatched positive fruits keys: {self.multi_frame._unmatched_positive_fruits.keys()}")

        if not peppers_temp:
            print(colored("No peppers here!", "blue"))
            return
        else:
            for key, value in peppers_temp.items():
                self.peppers = peppers_temp[key]
                print(f"Peppers, {self.peppers}")
                for v in value:
                    self.chosen_pepper = v
                    print(f"Chosen frame: {key}, Chosen pepper: {self.chosen_pepper.pepper_fruit.number}")
                    plot_frames(self.multi_frame, key, self.chosen_pepper.pepper_fruit.number)

                    try: 
                        if key in self.multi_frame._unmatched_positive_fruits.keys():
                            self.pepper_fruits = self.multi_frame._matched_positive_fruits[key] + self.multi_frame._unmatched_positive_fruits[key]
                            self.pepper_fruits.remove(self.chosen_pepper.pepper_fruit)
                            print(f"Pepper fruits after deleting chosen pepper: {self.pepper_fruits}")
                        else:
                            self.pepper_fruits = self.multi_frame._matched_positive_fruits[key]
                            self.pepper_fruits.remove(self.chosen_pepper.pepper_fruit)
                            print(f"Pepper fruits after deleting chosen pepper: {self.pepper_fruits}")
                    except Exception as e:
                        print(f"Key error while deleting pepper fruit from obstacles: {e}")
                        pass
                    return

        # self.set_pepper_order(arm_xyz)

    def send_to_manipulator(self):
        #################################################################
        # send the point of interaction to the manipulator over ROS
        #################################################################
        # print(self.peppers)
        # print(self.pepper_fruits)
        # print(self.pepper_peduncles)

        # if self.peppers:
        #     pepper = self.peppers.pop(0)
        #     poi = pepper.pepper_peduncle.poi
        #     del self.pepper_fruits[pepper.pepper_fruit.number]
        #     del self.pepper_peduncles[pepper.pepper_peduncle.number]
        #     print(self.peppers)
        #     print(self.pepper_fruits)
        #     print(self.pepper_peduncles)
        # else: 
        #     pepper = None
        #     print("No peppers left!")

        if self.chosen_pepper is None:
            self.multi_frame.clear_frames()
            self.multi_frame = MultiFrame()
            return 0
        else:
            print("00000000000000")
            print(self.chosen_pepper.pepper_peduncle)
            poi_in_base_link = self.chosen_pepper.pepper_peduncle.poi_in_base_link
            print("11111111111111111")
            print(poi_in_base_link)

            rate = rospy.Rate(10)
            start_time = time.time()
            while not rospy.is_shutdown() and time.time()- start_time < 20:
                # poi = self.chosen_pepper.pepper_peduncle.poi
                
                self.communication.publish_poi(poi_in_base_link, None)
                # self.communication.obstacle_pub_fn(self.pepper_fruits)
                # self.communication.rviz_marker_poi_base_link(self.peppers)
                # self.communication.rviz_marker(poi_in_base_link)
                # self.communication.rviz_marker_rs(poi_in_base_link)
                # self.communication.publish_poi([poi[0], poi[1], poi[2]], None)

                rate.sleep()

                self.multi_frame.clear_frames()
                self.multi_frame = MultiFrame()
                return 1
                # print(f"Number of frames in Multi-frame {len(self.multi_frame._one_frames)}")
                # print("publishing", list(self.peppers.values()))

    #####################################################################
    # VISUALIZATION related
    #####################################################################

    def send_to_gui(self):
        #################################################################
        # send information to gui over ros
        #################################################################
        pass

    def get_from_gui(self):
        #################################################################
        # get information from gui over ros
        # such as commands (stop running/change fps/etc)
        #################################################################
        pass
