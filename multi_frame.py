import itertools
from typing import List
from collections import deque

from one_frame import OneFrame
from pepper import Pepper
from pepper_fruit_utils import *


class MultiFrame:
    def __init__(self, max_frames=5):
        self._max_frames = max_frames
        self._one_frames = deque()
        self._positive_peppers: List[Pepper] = list()

    def add_one_frame(self, one_frame: OneFrame):
        if len(self._one_frames) == self._max_frames:
            self._one_frames.popleft()
        self._one_frames.append(one_frame)

    def clear_frames(self):
        return self._one_frames.clear()
    
    def populate_frames(self):
        for frame in self._one_frames:
            frame.run()

    def find_fruit_true_positives(self):
        fruit_detections_list = []
        for frame in self._one_frames:
            fruit_detections_list.append(frame.pepper_fruit_detections)
        
        combinations = list(itertools.combinations(fruit_detections_list, 2))

        associated_peppers = []

        for fruit_detections_1, fruit_detections_2 in combinations:
            for fruit_1 in fruit_detections_1.values():
                x, y, w, h = fruit_1.xywh
                box1 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]
                iou_list = []
                print(f"fruit 1: {fruit_1.number}")

                for fruit_2 in fruit_detections_2.values():
                    x, y, w, h = fruit_2.xywh
                    box2 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]
                    iou = calculate_iou(box1, box2)
                    iou_list.append((fruit_2, iou))

                iou_list.sort(key=lambda x: x[1], reverse=True)
                print(f"fruit 2: {iou_list[0][0].number}")

                if iou_list[0][1] > 0.3:  # Need to tune
                    fruit_1.occurences += 1
                    iou_list[0][0].occurences += 1
                    print(f"fruit 1: {fruit_1.number}, fruit 2: {iou_list[0][0].number}")

                    

        for fruit_detections in fruit_detections_list:
            for fruit in fruit_detections.values():
                if fruit.occurences > 0.5*len(fruit_detections_list):  # Need to tune
                    fruit.true_positive = True
                    self._positive_peppers.append(fruit)
                    print(f"fruit: {fruit.number}")