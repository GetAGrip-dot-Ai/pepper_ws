from typing import List
from PIL import Image
import torch
import numpy as np
import ultralytics
from ultralytics import YOLO
import matplotlib.pyplot as plt
import os

from pepper_peduncle import PepperPeduncle
from pepper_fruit_utils import print_pepperdetection,  read_image, draw_pepper_peduncles, draw_bounding_polygon


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
        self._predicted_peduncles = self.predict_peduncle(img_path, show_result, print_result, thresh=thresh)
        return self._predicted_peduncles
    
    def predict_peduncle(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.5):
        peduncle_dict = dict()

        img = read_image(img_path)
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

                peduncle_dict[i] = peduncle
                peduncle_count += 1

        # if print_result:
        #     print_result_masks(detected_frame)

        return peduncle_dict

    def predict_peduncles(self, show_result: bool = False, print_result: bool = False):

        for img_path in self._imgs_path:
            detected_frame = self.predict_peduncle(img_path, show_result, print_result)
            self._detected_frames.append(detected_frame)

    def plot_results(self, peduncle_list, poi_px, real_xyz):
        img = np.asarray(Image.open(self._imgs_path))
        img_name = self._imgs_path.split('/')[-1].split('.')[0]
        plt.imshow(img)
        for k, peduncle in peduncle_list.items():
            mask = peduncle.mask
            draw_bounding_polygon(peduncle.conf, mask, img.shape)
            poi_px = peduncle.poi_px
            plt.plot(poi_px[1], poi_px[0], 'ro', markersize=2)
        plt.text(poi_px[1], poi_px[0], f'{round(real_xyz[0],3),round(real_xyz[1],3),round(real_xyz[2],3)}')
        plt.plot(poi_px[1], poi_px[0], 'm*', markersize=5)
        plt.savefig(f"{os.getcwd()}/vs_result/{img_name}_peduncle_result.png")
        plt.clf()
        plt.cla()
        print(f"saved to: {os.getcwd()}/vs_result/{img_name}_peduncle_result.png")

if __name__ == '__main__':
    PepperPeduncleDetection = PepperPeduncleDetector(
        file_path='/dataset/peduncle',
        yolo_weight_path="../yolov8_scripts/weights/pepper_peduncle_best.pt")
    PepperPeduncleDetection.run_detection()
    print(PepperPeduncleDetection)
    PepperPeduncleDetection.plot_results()
