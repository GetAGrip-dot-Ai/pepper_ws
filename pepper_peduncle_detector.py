from typing import List
from PIL import Image
import torch
import time
import numpy as np
import ultralytics
from ultralytics import YOLO
import matplotlib.pyplot as plt
import os

from pepper_peduncle import PepperPeduncle
from pepper_fruit_utils import print_pepperdetection,  read_image, draw_pepper_peduncles, draw_bounding_polygon


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Wrapper class for a YOLOv8 model that outputs peduncle masks
"""


class PepperPeduncleDetector:
    def __init__(self, yolo_weight_path='weights/pepper_peduncle_best_4.pt'):

        # ultralytics.checks()

        self._model: YOLO = YOLO(yolo_weight_path)
        self._classes: List[str] = ["pepper"] # TODO if we change YOLO


    def __str__(self):
        return print_pepperdetection(self)
    
    def predict_peduncle(self, img_path, thresh=0.5):

        peduncle_dict = dict()
        # print("peduncle start reading")
        img = read_image(img_path)
        # print("peduncle img read")
        start_time = time.time()
        results = self._model(img, conf=thresh)
        # print("done YOLO:", time.time() - start_time)

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

        return peduncle_dict


    def plot_results(self, img_path, peduncle_list, poi_px, real_xyz):
        img = np.asarray(Image.open(img_path))
        img_name = img_path.split('/')[-1].split('.')[0]
        plt.imshow(img)
        plt.switch_backend('agg')
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
