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

    def add_one_frame(self, one_frame: OneFrame):
        if len(self._one_frames) == self._max_frames:
            self._one_frames.popleft()
        self._one_frames.append(one_frame)

    def add_video_frame(self, video_frame):
        if len(self._video_frames) == self._max_frames:
            self._video_frames.popleft()
        self._video_frames.append(video_frame)

    def video_frames_to_one_frames(self):
        number = 0
        for frame in self._video_frames:
            cv2.imwrite(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png', frame)
            self.add_one_frame(OneFrame(os.getcwd() + '/test_multi_frame/log/frame_' + str(number) + '.png'))
            number += 1

    def clear_frames(self):
        return self._one_frames.clear()
    
    def populate_frames(self):
        for frame in self._one_frames:
            frame.run()

    def populate_last_frame(self):
        self._one_frames[-1].run()

    def assign_frame_numbers(self):
        for i, frame in enumerate(self._one_frames):
            frame.frame_number = i

    def assign_last_frame_number(self):
        self._one_frames[-1].frame_number = len(self._one_frames) - 1

    def write_results(self):
        filename = os.getcwd() + '/test_multi_frame/log/results.txt'
        with open(filename, 'w') as f:
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

    # def get_pepper(self):
    #     if self._matched_positive_fruits:
    #         frame_number, peppers = next(iter(self._matched_positive_fruits.items()))
    #         pepper = peppers.pop()
