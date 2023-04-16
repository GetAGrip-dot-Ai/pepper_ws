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


class OneFrame:
    def __init__(self, img_path):
        self._frame_number: int = -1

        self._trans = None
        self._R = None
        # self.listener = tf.TransformListener()
    
        self.img_path = img_path  # should be a path to one image file

        self._img_shape: Tuple[int] = pepper_fruit_utils.get_img_size(img_path)

        self._mask: Optional[torch.Tensor] = None

        self.detected_pepper_fruit: bool = False
        self.detected_pepper_peduncle: bool = False

        self._pepper_fruit_count: int = 0
        self._pepper_peduncle_count: int = 0

        self._pepper_fruit_detections: Dict[int, PepperFruit] = dict()
        self._pepper_peduncle_detections: Dict[int, PepperPeduncle] = dict()
        self._pepper_detections: Dict[int, Pepper] = dict()
        self._pepper_fruit_detector: PepperFruitDetector = PepperFruitDetector(img_path,
                                 yolo_weight_path='weights/pepper_fruit_best_3.pt')
        self._pepper_peduncle_detector: PepperPeduncleDetector = PepperPeduncleDetector(img_path,
                                 yolo_weight_path='weights/pepper_peduncle_best_2.pt')

    @property
    def frame_number(self):
        return self._frame_number
    
    @frame_number.setter
    def frame_number(self, frame_number):
        self._frame_number = frame_number

    @property
    def trans(self):
        return self._trans

    @property
    def rot(self):
        return self._rot
    
    @property
    def img_shape(self):
        return self._img_shape

    @property
    def mask(self):
        return self._mask

    @mask.setter
    def mask(self, mask):
        self._mask = mask

    @property
    def pepper_fruit_detections(self):
        return self._pepper_fruit_detections

    @property
    def pepper_peduncle_detections(self):
        return self._pepper_peduncle_detections

    @property
    def pepper_fruit_count(self):
        return self._pepper_fruit_count

    @property
    def pepper_peduncle_count(self):
        return self._pepper_peduncle_count

    @property
    def pepper_detections(self):
        return self._pepper_detections

    def __str__(self):
        return f"DetectedFrame(frame={self.frame}, detections={self.detections})"

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

    def determine_pepper_xyz(self):
        for key, single_pepper in self._pepper_detections.items():
            x, y, z = get_depth(int(single_pepper.xywh[1]), int(single_pepper.xywh[0]))
            xyz = np.array([z, x, -y])
            single_pepper.pepper_fruit.xyz = xyz

    def determine_peduncle_poi(self):
        self.set_transform()
        for _, single_pepper in self._pepper_detections.items():
            single_pepper.pepper_peduncle.set_point_of_interaction(self._img_shape, single_pepper.pepper_fruit.xywh, self._trans, self._rot)


    def determine_peduncle_orientation(self):
        for _, single_pepper in self._pepper_detections.items():
            single_pepper.pepper_peduncle.set_peduncle_orientation(single_pepper.pepper_fruit.xywh)

    def set_transform(self):
        now = rospy.Time.now()
        if not self._trans:
            self.listener.waitForTransform("/rs_ee", "/base_link", now, rospy.Duration(10.0))
            (trans, rot) = self.listener.lookupTransform("/base_link", "/rs_ee", now)

            self._trans = np.array(trans).reshape(3, 1)
            self._R = R.from_quat([rot[0], rot[1], rot[2], rot[3]]).as_matrix() 
            self._trans = trans
            self._rot= rot
        else:
            self._trans = trans
            self._rot= rot

    def run(self):
        

        self._pepper_fruit_detections = self._pepper_fruit_detector.run_detection(self.img_path, thresh=0.3,
                                                show_result=False)
        # self.plot_pepper_fruit()
        self._pepper_peduncle_detections = self._pepper_peduncle_detector.run_detection(self.img_path, thresh=0.3,
                                                                                        show_result=False)
        # self.plot_pepper_peduncle()
        self.match_peppers()
        # self.determine_pepper_xyz()
        # print(self.frame_number)
        # print(self.pepper_fruit_detections)
        # self.plot_pepper()

        self.determine_peduncle_poi()
        # self.plot_poi()
        draw_all_multi_frame(self)
