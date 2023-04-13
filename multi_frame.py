import time
import itertools
from typing import List
from collections import deque

from one_frame import OneFrame
from pepper import Pepper
from pepper_fruit_utils import *
from multi_frame_utils import *


class MultiFrame:
    def __init__(self, max_frames=5):
        self._max_frames = max_frames
        self._one_frames = deque()
        self._video_frames = deque()

        self._matched_positive_fruits: Dict[int, List[PepperFruit]] = dict()
        self._unmatched_positive_fruits: Dict[int, List[PepperFruit]] = dict()
        self._matched_positive_peduncles: Dict[int, List[PepperPeduncle]] = dict()
        self._unmatched_positive_peduncles: Dict[int, List[PepperPeduncle]] = dict()
        self._matched_positive_peppers: Dict[int, List[Pepper]] = dict()

    def add_one_frame(self, one_frame: OneFrame):
        if len(self._one_frames) == self._max_frames:
            self._one_frames.popleft()
        self._one_frames.append(one_frame)

    # def add_video_frame(self, video_frame):
    #     if len(self._video_frames) == self._max_frames:
    #         self._video_frames.popleft()
    #     self._video_frames.append(video_frame)

    # def video_frames_to_one_frames(self):
    #     number = 0
    #     for frame in self._video_frames:
    #         cv2.imwrite(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png', frame)
    #         self.add_one_frame(OneFrame(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png'))
    #         number += 1

    def clear_frames(self):
        return self._one_frames.clear()
    
    def populate_frames(self):
        for frame in self._one_frames:
            frame.run()

    def assign_frame_numbers(self):
        for i, frame in enumerate(self._one_frames):
            frame.frame_number = i

    def write_results(self):
        filename = os.getcwd() + '/test_multi_frame/log/results.txt'
        with open(filename, 'a') as f:
            for key, value in self._matched_positive_fruits.items():
                f.write("Frame: " + str(key) + "\n")
                for v in value:
                    f.write("Matched Fruit: " + str(v.number) + "\n")

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
        

    def find_peduncles(self):
        combinations = list(itertools.combinations(self._one_frames, 2))

        for frame1, frame2 in combinations:
            update_peduncle_occurences(frame1.pepper_peduncle_detections.values(), frame2.pepper_peduncle_detections.values(), frame1.frame_number, frame2.frame_number)                    

        for frame in self._one_frames:
            update_peduncle_true_positives(frame.pepper_peduncle_detections.values(), len(self._one_frames))

        self._matched_positive_peduncles, self._unmatched_positive_peduncles = get_all_peduncles(self._one_frames)


    def find_peppers(self):
        for frame_number, fruits in self._matched_positive_fruits.items():
            for fruit in fruits:
                peduncle = self._one_frames[frame_number].pepper_detections[fruit.parent_pepper].pepper_peduncle
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
        self.assign_frame_numbers()
        self.populate_frames()
        self.find_fruits()
        self.find_peduncles()
        self.write_results()

        self.find_peppers()
        self.write_results()
