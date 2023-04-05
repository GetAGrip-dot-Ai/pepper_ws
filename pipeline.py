# sys.path.insert(0, '/home/shri/Perception_Resources/yolov8_scripts/src')
import os
import time

import matplotlib.pyplot as plt
import rospy
import time

print(os.getcwd())

from one_frame import OneFrame
from communication import Communication
from pepper_fruit_utils import *


# input: image
class Perception:
    def __init__(self, source, fps, threshold=0.5, percentage=0.5, save=True):
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
        self.communication = Communication()

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

    def process_image(self):
        #################################################################
        # DO_NOT_DO
        # process the image to match the dim for yolo (prob don't need this)
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
        self.one_frame.run_realtime()
        self.pepper_fruits = self.one_frame.pepper_fruit_detections
        self.pepper_peduncles = self.one_frame.pepper_peduncle_detections
        self.peppers = self.one_frame.pepper_detections
        print(self.pepper_peduncles)
        print(self.pepper_peduncles[0].poi)
        return self.pepper_peduncles[0].poi
    def detect_peppers_realtime_frame(self, img, thresh=0.5):
        #################################################################
        # use yolov8_scripts and get the pepper locations
        # input:
        #   path: image path
        #   thresh: disgard detection lower than threshold
        # output:
        #   locations: all the locations of the pepper boxes [conf, x, y] (N, 3)
        #################################################################
        # self.one_frame = RealTimeFrame(img)
        self.one_frame.run()
        self.pepper_fruits = self.one_frame.pepper_fruit_detections
        self.pepper_peduncles = self.one_frame.pepper_peduncle_detections
        self.peppers = self.one_frame.pepper_detections

    def detect_peppers_in_folder(self):
        files = get_all_image_path_in_folder(self.source)
        print(os.getcwd(),"==", self.source)
        print("wt:", files)
        for path in files:
            self.detect_peppers_one_frame(path)
            
    def detect_peppers_realtime(self):
        while True:
            user_input = input("1: start\n2: end\n")
            if user_input == "1":
                time.sleep(2)
                print("taking pic!")
                img = get_image_from_webcam()
                img_name=str(time.time()).split('.')[0]
                cv2.imwrite(os.getcwd()+'/realtime/'+img_name+'.png', img)
                print("saved to :", os.getcwd()+'/realtime/'+img_name+'.png')
                (poi_x, poi_y, poi_z) = self.detect_peppers_one_frame(os.getcwd()+'/realtime/'+img_name+'.png')
                return (poi_x, poi_y)
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
        self.one_frame.determine_pepper_order(arm_xyz)

    #####################################################################
    # ROS related
    #####################################################################

    def send_to_manipulator(self):
        #################################################################
        # send the point of interaction to the manipulator over ROS
        #################################################################
        print("ros_peppers", self.peppers.values())

        pepper = list(self.peppers.values())[0]
        poi = pepper.pepper_peduncle.poi
        print("POI: ", poi)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.communication.obstacle_pub_fn(list(self.pepper_fruits.values()))
            # self.communication.poi_pub_fn([poi[0], poi[1], poi[2]], None)
            rate.sleep()

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
