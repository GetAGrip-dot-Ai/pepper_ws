from typing import List, Optional


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class for a pepper fruit and its properties
"""


class PepperFruit:
    def __init__(self, number:int, xywh=None, conf=0.0):
        self._number: int = number

        self._xywh: Optional[List[float]] = xywh
        self._conf: float = conf
        self._xyz = None
        self._true_positive: bool = False
        self._occurences: int = 1
        self._associated_fruits: List[(int, PepperFruit)] = list()
        self._parent_pepper: int = None

    @property
    def number(self):
        return self._number

    @property
    def xywh(self):
        return self._xywh

    @xywh.setter
    def xywh(self, xywh):
        self._xywh = xywh

    @property
    def conf(self):
        return self._conf

    @conf.setter
    def conf(self, conf):
        self._conf = conf

    @property
    def xyz(self):
        return self._xyz
    
    @xyz.setter
    def xyz(self, xyz):
        self._xyz = xyz

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
    def associated_fruits(self):
        return self._associated_fruits
    
    def add_associated_fruit(self, frame_number, fruit):
        self._associated_fruits.append((frame_number, fruit))

    def __str__(self):
        return f"Pepper(number={self.number}, xywh={self.xywh}, conf={self._conf})"
