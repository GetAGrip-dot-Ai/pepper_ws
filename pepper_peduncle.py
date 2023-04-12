import os
from typing import List
from realsense_utils import *
from pepper_peduncle_utils import *

class PepperPeduncle:
    def __init__(self, number: int, mask=None, conf=None, percentage=0.5):
        self.number: int = number
        self._mask = mask
        self._conf: float = conf
        self._percentage = percentage
        self._xywh = None
        self._curve = Curve()
        self._poi = None
        self._poi_px = None
        self._orientation = [1, 0, 0]
        self._true_positive: bool = False
        self._occurences: int = 1
        self._associated_peduncles: List[(int, PepperPeduncle)] = list()
        self._parent_pepper: int = None
    
    @property
    def mask(self):
        return self._mask

    @mask.setter
    def mask(self, mask):
        self._mask = mask

    @property
    def conf(self):
        if self._conf is None:
            return 0
        return self._conf

    @conf.setter
    def conf(self, conf):
        self._conf = conf

    @property
    def xywh(self):
        return self._xywh

    @xywh.setter
    def xywh(self, value):
        self._xywh = value

    @property
    def curve(self):
        return self._curve

    @curve.setter
    def curve(self, curve):
        self._curve = curve

    @property
    def poi(self):
        return self._poi

    @poi.setter
    def poi(self, value):
        self._poi = value

    @property
    def orientation(self):
        return self._orientation
    
    @property
    def poi_px(self):
        return self._poi_px

    @orientation.setter
    def orientation(self, value):
        self._orientation = value

    @property
    def true_positive(self):
        return self._true_positive
    
    @true_positive.setter
    def true_positive(self, true_positive):
        self._true_positive = true_positive

    @property
    def occurences(self):
        return self._occurences
    
    @occurences.setter
    def occurences(self, occurences):
        self._occurences = occurences

    @property
    def parent_pepper(self):
        return self._parent_pepper
    
    @parent_pepper.setter
    def parent_pepper(self, parent_pepper):
        self._parent_pepper = parent_pepper

    @property
    def associated_peduncles(self):
        return self._associated_peduncles
    
    def add_associated_peduncle(self, frame_number, peduncle):
        self._associated_peduncles.append((frame_number, peduncle))

    def __str__(self):
        return f"Peduncle(number={self.number},mask={self._mask}, conf={self._conf})"

    def set_point_of_interaction(self, img_shape, pepper_fruit_xywh):
        # plt.savefig(f"{os.getcwd()}/result/hi.png")
        self._curve = fit_curve_to_mask(self._mask, img_shape, pepper_fruit_xywh, self._xywh)
        total_curve_length = self._curve.full_curve_length()

        poi_x_px, poi_y_px = determine_poi(self._curve, self._percentage, total_curve_length)
        # poi_x, poi_y, poi_z = get_depth(int(poi_x_px), int(poi_y_px))

        # self._poi = (poi_z, poi_x, -poi_y)
        self._poi_px = (poi_x_px, poi_y_px)
        # print("POI in world frame:", poi_x, poi_y, poi_z)
        print("POI in pixel frame:", self._poi_px)
              
    def set_peduncle_orientation(self, pepper_fruit_xywh):
        point_x, point_y = determine_next_point(self._curve, self._poi, pepper_fruit_xywh, self._xywh)
        point_z = get_depth(point_x, point_y)
        self._orientation = [point_x - self._poi[0], point_y - self._poi[1], point_z - self._poi[2]]


