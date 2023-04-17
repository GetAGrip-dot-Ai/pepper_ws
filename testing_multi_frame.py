#!/usr/bin/env python3
from pepper_fruit import PepperFruit
from one_frame import OneFrame
from multi_frame import MultiFrame
from multi_frame_utils import *
from realsense_utils import *
import cv2
import os

if __name__ == '__main__':
    obj = MultiFrame(10)

    """
    This is for testing on images
    """
    # start = 52
    # for i in range(2):
    #     filename = f"/home/shri/pepper_ws/test_multi_frame/IMG_08{start + i}.jpg"
    #     obj.add_one_frame(OneFrame(filename))

    """
    This is for testing on a saved video
    """
    # cap = cv2.VideoCapture('/home/shri/pepper_ws/test_multi_frame/IMG_1254.mp4')
    # while cap.isOpened():
    #     ret, frame = cap.read()
    #     if ret:
    #         obj.add_video_frame(frame)
    #     else:
    #         break

    # cap.release()

    """
    This is for testing on webcam with continuous input
    """
    # cap = cv2.VideoCapture(0)
    # while True:
    #     ret, frame = cap.read()
    #     cv2.imshow('frame', frame)
    #     obj.add_video_frame(frame)
        
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
        
    # cap.release()
    # cv2.destroyAllWindows()

    # obj.video_frames_to_one_frames()

    # print(len(obj._one_frames))

    # obj.assign_frame_numbers()
    # obj.populate_frames()
    # obj.find_fruits()
    # obj.find_peduncles()
    # obj.write_results()

    """
    This is for testing on webcam with discontinuous input
    """
    # user_input = input("Enter 1 to take picture and 2 to start multi-frame processing!")
    number = 0

    # while user_input == "1":
    while number != 10:
        # img = get_image()
        # cv2.imwrite(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png', img)
        obj.add_one_frame(OneFrame(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png'))
        obj.assign_last_frame_number(number)
        number += 1
        # user_input = input("Enter 1 to take picture and 2 to start multi-frame processing!")
    
    # obj.populate_frames()
    # obj.run()




    """
    This code is used for testing on fake bounding box data.
    """
    # f1 = OneFrame("")
    # f2 = OneFrame("")
    # f3 = OneFrame("")

    # f1.pepper_fruit_detections[0] = PepperFruit(0, [20,20,10,10], 0.8)
    # f1.pepper_fruit_detections[1] = PepperFruit(1, [50,50,10,10], 0.8)

    # f2.pepper_fruit_detections[0] = PepperFruit(2, [22,22,10,10], 0.8)
    # f2.pepper_fruit_detections[1] = PepperFruit(3, [47,47,10,10], 0.8)

    # f3.pepper_fruit_detections[0] = PepperFruit(4, [21,24,10,10], 0.8)

    # obj.add_one_frame(f1)
    # obj.add_one_frame(f2)
    # obj.add_one_frame(f3)

    # obj.assign_frame_numbers()

    # obj.find_fruits()