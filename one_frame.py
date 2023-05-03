from typing import Optional, Tuple

from pepper import Pepper
from pepper_fruit_detector import PepperFruitDetector
from pepper_peduncle_detector import PepperPeduncleDetector
from pepper_fruit_utils import *
from realsense_utils import *
from pepper_utils import *
import os
import tf
import rospy
from scipy.spatial.transform import Rotation as R
from termcolor import colored
from communication import Communication


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class that processes a single image frame to output pepper bounding boxes, peduncle masks and POIs
"""


class OneFrame:
    def __init__(self, img_path, number, yolo_weights_pf_path = 'weights/pepper_fruit_best_4.pt', yolo_weights_pp_path='weights/pepper_peduncle_best_4.pt'):
        self._frame_number: int = number

        self._trans = None
        self._R = None
        self.listener = tf.TransformListener()
    
        self.img_path = img_path  # should be a path to one image file

        self._img_shape: Tuple[int] = pepper_fruit_utils.get_img_size(img_path)

        # self._mask: Optional[torch.Tensor] = None

        # self.detected_pepper_fruit: bool = False
        # self.detected_pepper_peduncle: bool = False

        # self._pepper_fruit_count: int = 0
        # self._pepper_peduncle_count: int = 0

        self._pepper_fruit_detections: Dict[int, PepperFruit] = dict()
        self._pepper_peduncle_detections: Dict[int, PepperPeduncle] = dict()
        self._pepper_detections: Dict[int, Pepper] = dict()

        self._pepper_fruit_detector: PepperFruitDetector = PepperFruitDetector(yolo_weight_path=yolo_weights_pf_path)
        self._pepper_peduncle_detector: PepperPeduncleDetector = PepperPeduncleDetector(yolo_weight_path=yolo_weights_pp_path)
        # TODO: check PepperFruitDetector
    
    @property
    def frame_number(self):
        return self._frame_number
    
    # @frame_number.setter
    # def frame_number(self, frame_number):
    #     self._frame_number = frame_number

    @property
    def trans(self):
        return self._trans

    @property
    def rot(self):
        return self._rot
    
    @property
    def img_shape(self):
        return self._img_shape

    # @property
    # def mask(self):
    #     return self._mask

    # @mask.setter
    # def mask(self, mask):
    #     self._mask = mask

    @property
    def pepper_fruit_detections(self):
        return self._pepper_fruit_detections

    @property
    def pepper_peduncle_detections(self):
        return self._pepper_peduncle_detections

    # @property
    # def pepper_fruit_count(self):
    #     return self._pepper_fruit_count

    # @property
    # def pepper_peduncle_count(self):
    #     return self._pepper_peduncle_count

    @property
    def pepper_detections(self):
        return self._pepper_detections

    def __str__(self): # TODO : maybe see what it is printing
        return f"DetectedFrame(frame={self.frame}, detections={self._pepper_detections})"

    def match_peppers(self):
        pepper_fruit_peduncle_match = match_pepper_fruit_peduncle(self._pepper_fruit_detections,
                                                                  self._pepper_peduncle_detections)
        number = 0
        for (pfn, ppn), _ in pepper_fruit_peduncle_match:
            if ppn == -1:
                continue
            else:
                pepper = Pepper(number, pfn, ppn)
                pepper.pepper_fruit = self._pepper_fruit_detections[pfn]
                pepper.pepper_fruit.parent_pepper = number
                pepper.pepper_peduncle = self.pepper_peduncle_detections[ppn]
                pepper.pepper_peduncle.parent_pepper = number
                self._pepper_detections[number] = pepper
                number += 1

    def determine_pepper_fruit_xyz(self, rs_camera):
        comm = Communication()
        for key, single_pepper_fruit in self._pepper_fruit_detections.items():
            x, y, z = get_depth(rs_camera, int(single_pepper_fruit.xywh[1]), int(single_pepper_fruit.xywh[0]))
            xyz = np.array([z + 0.06, x, -y])
            single_pepper_fruit.xyz = comm.transform_to_base_link( xyz, self._trans, self._rot)

    def determine_peduncle_poi(self, rs_camera):
        self.set_transform()
        for _, single_pepper in self._pepper_detections.items():
            single_pepper.pepper_peduncle.set_point_of_interaction(self._img_shape, rs_camera, single_pepper.pepper_fruit.xywh, self._trans, self._rot)

    def determine_peduncle_orientation(self):
        for _, single_pepper in self._pepper_detections.items():
            single_pepper.pepper_peduncle.set_peduncle_orientation(single_pepper.pepper_fruit.xywh)

    def set_transform(self):

        if not self._trans:

            now = rospy.Time.now()

            self.listener.waitForTransform("/realsense_frame", "/base_link", now, rospy.Duration(10.0)) # TODO do we have to wait? CHECK IN REAL LIFE
            (trans, rot) = self.listener.lookupTransform("/base_link", "/realsense_frame", now)

            self._trans = np.array(trans).reshape(3, 1)
            self._R = R.from_quat([rot[0], rot[1], rot[2], rot[3]]).as_matrix() 
            self._trans = trans
            self._rot= rot
            print(colored("TRANSFORM SUCCESS", 'blue'))
        else:
            return

    def run(self, rs_camera):
        self._pepper_fruit_detections = self._pepper_fruit_detector.predict_pepper(self.img_path, thresh=0.3)
        # self.plot_pepper_fruit()
        self._pepper_peduncle_detections = self._pepper_peduncle_detector.predict_peduncle(self.img_path, thresh=0.3)
        # self.plot_pepper_peduncle()
        self.match_peppers()

        self.determine_peduncle_poi(rs_camera)

        draw_all_multi_frame(self)
