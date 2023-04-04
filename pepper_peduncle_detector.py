from typing import List

import torch
import numpy as np
import ultralytics
from ultralytics import YOLO
import matplotlib.pyplot as plt

from pepper_peduncle import PepperPeduncle
from pepper_fruit_utils import print_pepperdetection, get_all_image_path_in_folder, read_image, \
    draw_pepper_peduncles


class PepperPeduncleDetector:
    def __init__(self, file_path, yolo_weight_path, img=None):

        ultralytics.checks()

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
        self._imgs_path = get_all_image_path_in_folder(self._path)
        # self.predict_peduncles(show_result, print_result)
        return self.predict_peduncle(img_path, show_result, print_result, thresh=thresh)
    
    def run_detection_realtime(self, show_result: bool = False, print_result: bool = False, thresh=0.5):
        self._imgs_path = get_all_image_path_in_folder(self._path)
        # self.predict_peduncles(show_result, print_result)
        return self.predict_peduncle_realtime(show_result, print_result, thresh=thresh)

    def predict_peduncle(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.5):
        peduncle_list = dict()
        # print("Detecting image: ", img_path)

        img = read_image(img_path)
        results = self._model(img, conf=thresh)
        peduncle_count = 0

        result = results[0]
        if result.boxes.boxes.size(0) != 0:
            for i in range(result.masks.shape[0]):
                mask = result.masks
                box = result.boxes  # Boxes object for bbox outputs

                peduncle = PepperPeduncle(peduncle_count)
                # print("11111111111111111111111111111111")
                # print(result.masks)

                peduncle.mask = torch.Tensor(mask.segments[i])
                # print("mask_from_detector", peduncle.mask)
                # print(peduncle.mask.size())
                # # peduncle.segment = mask.data.reshape((mask.shape[1], mask.shape[2]))
                # peduncle.segment = np.zeros((img.shape[0], img.shape[1]))
                # print(peduncle.segment.shape)
                # for i in range(peduncle.mask.size()[0]):
                #     print('i,', i)
                #     x, y = peduncle.mask[i, :]
                #     print(x, y)
                #     peduncle.segment[int(peduncle.mask[i, 0]), int(peduncle.mask[i, 1])] = 1
                # print("==========", peduncle.mask.shape)
                # peduncle.segment = mask.data.T
                # plt.imshow(peduncle.segment)
                # plt.savefig("please.png")
                peduncle.conf = box.conf[i]
                peduncle.xywh = box.xywh[i].cpu().numpy()

                peduncle_list[i] = peduncle
                peduncle_count += 1
        # detected_frame.mask = result.masks.masks
        if show_result:
            for result in results:
                res_plotted = result[0].plot()
                # cv2.imshow("result", res_plotted)

        # if print_result:
        #     print_result_masks(detected_frame)

        return peduncle_list
    def predict_peduncle_realtime(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.5):
        peduncle_list = dict()
        # print("Detecting image: ", img_path)

        img = self._img
        results = self._model(img, conf=thresh)
        peduncle_count = 0

        result = results[0]
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
