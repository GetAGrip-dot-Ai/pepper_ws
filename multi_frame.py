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
        self._positive_peppers: Dict[int, List[Pepper]] = dict()

    def add_one_frame(self, one_frame: OneFrame):
        if len(self._one_frames) == self._max_frames:
            self._one_frames.popleft()
        self._one_frames.append(one_frame)

    def clear_frames(self):
        return self._one_frames.clear()
    
    def populate_frames(self):
        for frame in self._one_frames:
            frame.run()

    def assign_frame_numbers(self):
        for i, frame in enumerate(self._one_frames, 1):
            frame.frame_number = i

    def find_fruits(self):
        combinations = list(itertools.combinations(self._one_frames, 2))

        for frame1, frame2 in combinations:
            update_fruit_occurences(frame1.pepper_fruit_detections.values(), frame2.pepper_fruit_detections.values(), frame1.frame_number, frame2.frame_number)                    

        for frame in self._one_frames:
            update_true_positives(frame.pepper_fruit_detections.values(), len(self._one_frames))

        for frame in self._one_frames:
            self._positive_peppers = get_all_fruits(self._one_frames)

        for key, value in self._positive_peppers.items():
            print(f"Frame {key}")
            for v in value:
                print(f"Fruit {v.number}")