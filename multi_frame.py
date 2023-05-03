import time
import itertools
from typing import List
from collections import deque

from one_frame import OneFrame
from pepper import Pepper
from pepper_fruit_utils import *
from multi_frame_utils import *
from termcolor import colored


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class with algorithm for matching peppers across different frames to find true positive peppers
"""


class MultiFrame:
    def __init__(self, max_frames=10):
        self._max_frames = max_frames
        self._one_frames = deque()
        self._video_frames = deque()

        self._matched_positive_fruits: Dict[int, List[PepperFruit]] = dict()
        self._unmatched_positive_fruits: Dict[int, List[PepperFruit]] = dict()
        self._matched_positive_peduncles: Dict[int, List[PepperPeduncle]] = dict()
        self._unmatched_positive_peduncles: Dict[int, List[PepperPeduncle]] = dict()
        self._matched_positive_peppers: Dict[int, List[PepperPeduncle]] = dict()

    def add_one_frame(self, one_frame: OneFrame):
        if len(self._one_frames) == self._max_frames:
            self._one_frames.popleft()
        self._one_frames.append(one_frame)

    def clear_frames(self):
        return self._one_frames.clear()
    
    def populate_frames(self):
        for frame in self._one_frames:
            frame.run()

    def populate_last_frame(self, rs_camera):
        self._one_frames[-1].run(rs_camera)

    def assign_frame_numbers(self):
        for i, frame in enumerate(self._one_frames):
            frame.frame_number = i

    def write_results(self):
        filename = os.getcwd() + '/test_multi_frame/log/results.txt'
        with open(filename, 'w') as f:
            for key, value in self._matched_positive_fruits.items():
                f.write("Frame: " + str(key) + "\n")
                for v in value:
                    f.write("Matched Fruit: " + str(v.number) + " ")
                    f.write("Associated Fruits: ")
                    for frame_number, associated_fruit in v.associated_fruits:
                        f.write(str(frame_number) + " " + str(associated_fruit) + ", ")
                    f.write("\n")

            for key, value in self._unmatched_positive_fruits.items():
                f.write("Frame: " + str(key) + "\n")
                for v in value:
                    f.write("Unmatched Fruit: " + str(v.number) + "\n")

            for key, value in self._matched_positive_peduncles.items():
                f.write("Frame: " + str(key) + "\n")
                for v in value:
                    f.write("Matched Peduncle: " + str(v.number) + "\n")

            for key, value in self._unmatched_positive_peduncles.items():
                f.write("Frame: " + str(key) + "\n")
                for v in value:
                    f.write("Unmatched Peduncle: " + str(v.number) + "\n")

            f.write("----------------------------------------\n")
            

    def find_fruits(self):
        combinations = list(itertools.combinations(self._one_frames, 2))

        for frame1, frame2 in combinations:
            update_fruit_occurences(frame1.pepper_fruit_detections.values(), frame2.pepper_fruit_detections.values(), frame1.frame_number, frame2.frame_number)                    

        for frame in self._one_frames:
            update_fruit_true_positives(frame.pepper_fruit_detections.values(), len(self._one_frames))

        self._matched_positive_fruits, self._unmatched_positive_fruits = get_all_fruits(self._one_frames)

        print(colored(f"matched fruit: {self._matched_positive_fruits}", "yellow"))
        print(colored(f"unmatched fruit: {self._unmatched_positive_fruits}", "yellow"))
        

    def find_peduncles(self):
        combinations = list(itertools.combinations(self._one_frames, 2))

        for frame1, frame2 in combinations:
            update_peduncle_occurences(frame1.pepper_peduncle_detections.values(), frame2.pepper_peduncle_detections.values(), frame1.frame_number, frame2.frame_number)                    

        for frame in self._one_frames:
            update_peduncle_true_positives(frame.pepper_peduncle_detections.values(), len(self._one_frames))

        self._matched_positive_peduncles, self._unmatched_positive_peduncles = get_all_peduncles(self._one_frames)


    def find_peppers(self): # TODO: make this more efficient? if we have time 

        for frame_number, fruits in self._matched_positive_fruits.items():
            self._matched_positive_peppers[frame_number] = []

            for fruit in fruits:

                parent_pepper = self._one_frames[frame_number].pepper_detections[fruit.parent_pepper]
                self._matched_positive_peppers[frame_number].append(parent_pepper)

                peduncle = parent_pepper.pepper_peduncle

                if frame_number in self._matched_positive_peduncles and peduncle in self._matched_positive_peduncles[frame_number]:

                    peduncles = self._matched_positive_peduncles[frame_number]
                    peduncles.remove(peduncle)

                    if peduncles:
                        self._matched_positive_peduncles[frame_number] = peduncles
                    else:
                        del self._matched_positive_peduncles[frame_number]

                for associated_fruit_frame_number, associated_fruit_number in fruit.associated_fruits:
                    associated_fruit_parent_number = self._one_frames[associated_fruit_frame_number].pepper_fruit_detections[associated_fruit_number].parent_pepper
                    if associated_fruit_parent_number is not None:
                        associated_peduncle = self._one_frames[associated_fruit_frame_number].pepper_detections[associated_fruit_parent_number].pepper_peduncle

                        if associated_fruit_frame_number in self._matched_positive_peduncles and associated_peduncle in self._matched_positive_peduncles[associated_fruit_frame_number]:
                            peduncles = self._matched_positive_peduncles[associated_fruit_frame_number]
                            peduncles.remove(associated_peduncle)
                            if peduncles:
                                self._matched_positive_peduncles[associated_fruit_frame_number] = peduncles
                            else:
                                del self._matched_positive_peduncles[associated_fruit_frame_number]

    def run(self):
        # self.assign_frame_numbers()
        self.find_fruits()
        self.find_peduncles()

        self.find_peppers()
        self.write_results()
