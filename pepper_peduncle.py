import os
from typing import List
from realsense_utils import *
from pepper_peduncle_utils import *
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class for a pepper peduncle and its properties
"""


class PepperPeduncle:
    def __init__(self, number: int, mask=None, conf=None, percentage=0.42):
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
        self._poi_in_base_link = None
    
    @property
    def poi_in_base_link(self):
        return self._poi_in_base_link

    @poi_in_base_link.setter
    def poi_in_base_link(self, poi_in_base_link):
        self._poi_in_base_link = poi_in_base_link

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

    def set_point_of_interaction(self, img_shape, rs_camera, pepper_fruit_xywh=None, trans=None, rot=None):

        if pepper_fruit_xywh is None:  
            pepper_fruit_xywh = self._xywh
            pepper_fruit_xywh[1] = pepper_fruit_xywh[1] - 2

        self._curve = fit_curve_to_mask(self._mask, img_shape, pepper_fruit_xywh, self._xywh)
        total_curve_length = self._curve.full_curve_length()

        poi_x_px, poi_y_px = determine_poi(self._curve, self._percentage, total_curve_length)

        poi_x, poi_y, poi_z = get_depth(rs_camera, int(poi_x_px), int(poi_y_px))

        self._poi = (poi_z, -poi_x, -poi_y) # converting to kinova's axis frame
        self._poi_px = (poi_x_px, poi_y_px)

        self.get_poi_in_base_link(trans, rot)
        # print("POI in world frame:", poi_x, poi_y, poi_z)
        # print("POI in pixel frame:", self._poi_px)


    def set_point_of_interaction_orig(self, img_shape, pepper_fruit_xywh=None, trans=None, rot=None):

        if pepper_fruit_xywh is None:  
            pepper_fruit_xywh = self._xywh
            pepper_fruit_xywh[1] = pepper_fruit_xywh[1] - 2

        self._curve = fit_curve_to_mask(self._mask, img_shape, pepper_fruit_xywh, self._xywh)
        total_curve_length = self._curve.full_curve_length()

        poi_x_px, poi_y_px = determine_poi(self._curve, self._percentage, total_curve_length)
        self._poi_px = (poi_x_px, poi_y_px)
        

    def get_poi_in_base_link(self, trans, rot):
        try:

            point_in_relative_frame = self._poi

            r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
            H = np.vstack((np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1))),np.array([0,0,0,1])))
            point_in_relative_frame = list(point_in_relative_frame) + [1]
            point = np.array(H) @ np.array(point_in_relative_frame).T

            self.poi_in_base_link = point
            print(colored(f"POI relative to the base_link:\n{point}", "yellow"))
        except Exception as e: # TODO needed only for visual servo?
            
            print(colored(f"No tf found: {e}", "magenta"))


    def set_peduncle_orientation(self, rs_camera, pepper_fruit_xywh):
        point_x, point_y = determine_next_point(self._curve, self._poi, pepper_fruit_xywh, self._xywh)
        point_z = get_depth(rs_camera, point_x, point_y)
        self._orientation = [point_x - self._poi[0], point_y - self._poi[1], point_z - self._poi[2]]


