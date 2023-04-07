from typing import List

from pepper_ws.one_frame import OneFrame
from pepper_ws.pepper import Pepper
from ros_kortex.pepper_ws.pepper_fruit_utils import *


class MultiFrame:
    def __init__(self):
        self._one_frames: List[OneFrame] = list()
        self._positive_peppers: List[Pepper] = list()


box_1 = [[511, 41], [577, 41], [577, 76], [511, 76]]
box_2 = [[544, 59], [610, 59], [610, 94], [544, 94]]

print(calculate_iou(box_1, box_2))
