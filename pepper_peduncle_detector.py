from typing import List

import torch
import numpy as np
import ultralytics
from ultralytics import YOLO
import matplotlib.pyplot as plt

from pepper_peduncle import PepperPeduncle
from pepper_fruit_utils import print_pepperdetection,  read_image, draw_pepper_peduncles


class PepperPeduncleDetector:
    def __init__(self, file_path, yolo_weight_path, img=None):

        ultralytics.checks()
        print("yolo path is:", yolo_weight_path)
        self._model: YOLO = YOLO(yolo_weight_path)
        self._path: str = file_path
        self._classes: List[str] = ["pepper"]

        self._imgs_path: List[str] = list()
        self._img = img
        # self._detected_frames: List[OneFrame] = list()

    @property
    def detected_frames(self):
        return self._detected_frames

    @detected_frames.setter
    def detected_frames(self, detected_frames):
        self._detected_frames = detected_frames

    @property
    def path(self):
        return self._path

    def __str__(self):
        return print_pepperdetection(self)

    def run_detection(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.5):
        # self._imgs_path = get_all_image_path_in_folder(self._path)
        self._imgs_path = img_path
        # self.predict_peduncles(show_result, print_result)
        return self.predict_peduncle(img_path, show_result, print_result, thresh=thresh)
    
    def predict_peduncle(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.5):
        peduncle_list = dict()
        print("Detecting image: ", img_path)

        img = read_image(img_path)
        results = self._model(img, conf=thresh)
        # print("results: ", results)
        # print("predict peduncle")
        peduncle_count = 0

        result = results[0]
        # print("results:", results)
        if result.boxes.boxes.size(0) != 0:
            for i in range(result.masks.shape[0]):
                mask = result.masks
                box = result.boxes  # Boxes object for bbox outputs

                peduncle = PepperPeduncle(peduncle_count)

                peduncle.mask = torch.Tensor(mask.segments[i])

                peduncle.conf = box.conf[i]
                peduncle.xywh = box.xywh[i].cpu().numpy()

                peduncle_list[i] = peduncle
                peduncle_count += 1

        # if show_result:
        #     for result in results:
        #         res_plotted = result[0].plot()
                # cv2.imshow("result", res_plotted)

        # if print_result:
        #     print_result_masks(detected_frame)
        # print("peduncle_list: ", peduncle_list)
        return peduncle_list

    def predict_peduncles(self, show_result: bool = False, print_result: bool = False):

        for img_path in self._imgs_path:
            detected_frame = self.predict_peduncle(img_path, show_result, print_result)
            self._detected_frames.append(detected_frame)

    def plot_results(self):
        for detected_img in self.detected_frames:
            draw_pepper_peduncles(detected_img)


if __name__ == '__main__':
    PepperPeduncleDetection = PepperPeduncleDetector(
        file_path='/dataset/peduncle',
        yolo_weight_path="../yolov8_scripts/weights/pepper_peduncle_best.pt")
    PepperPeduncleDetection.run_detection()
    print(PepperPeduncleDetection)
    PepperPeduncleDetection.plot_results()
