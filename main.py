#!/usr/bin/env python3
import rospy 
from pipeline import Perception
import os

if __name__ == '__main__':
    # os.chdir(os.getcwd()+'/pepper_ws/')
    os.chdir('/'.join(__file__.split('/')[:-1]))
    print("current working dir: ",os.getcwd())
    # img_path = '../dataset/testbed_video_to_img'
    test_img_path = '/test_single'
    pipeline = Perception(test_img_path, 0)
    pipeline.detect_peppers_in_folder()
    pipeline.send_to_manipulator()

'''
input an image
    make a one_frame
        run pepper_fruit detection
        run pepper_peduncle detection
        match pepper
    get peduncle location 

'''
