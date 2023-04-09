from typing import List, Optional


class PepperFruit:
    def __init__(self, number:int, xywh=None, conf=0.0):
        self._number: int = number

        self._xywh: Optional[List[float]] = xywh
        self._conf: float = conf
        self._true_positive: bool = False
        self._occurences: int = 1
        self._associated_peppers: List[(int, PepperFruit)] = list()

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
    def associated_peppers(self):
        return self._associated_peppers
    
    def add_associated_pepper(self, frame_number, pepper):
        self._associated_peppers.append((frame_number, pepper))

    def __str__(self):
        return f"Pepper(number={self.number}, xywh={self.xywh}, conf={self._conf})"
