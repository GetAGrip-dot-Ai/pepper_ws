from typing import List

import ultralytics
from ultralytics import YOLO

from pepper_fruit import PepperFruit
from pepper_fruit_utils import print_pepperdetection, get_all_image_path_in_folder, read_image, print_result_boxes, remove_overlapping_boxes


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Wrapper class that creates a YOLOv8 model and provides peppers detected
"""


class PepperFruitDetector: # TODO: why is it initializing for every one frame?
    def __init__(self, yolo_weight_path: str):

        # ultralytics.checks()

        self._model: YOLO = YOLO(yolo_weight_path)
        self._classes: List[str] = ["pepper"] # TODO if we change YOLO

    def __str__(self):
        return print_pepperdetection(self)

    def predict_pepper(self, img_path, thresh=0.25):

        pepper_dict = dict()

        img = read_image(img_path)

        results = self._model(img, conf=thresh)
        
        pepper_count = 0

        for result in results:
            boxes = result.boxes  # Boxes object for bbox outputs
            for box in boxes:
                one_box = box[0]
                pepper = PepperFruit(pepper_count)
                xywh = one_box.xywh
                conf = one_box.conf  # Class probabilities for classification outputs
                pepper.xywh = xywh[0].cpu().numpy()
                pepper.conf = conf
                pepper_dict[pepper_count] = pepper
                pepper_count += 1

        pepper_dict = remove_overlapping_boxes(pepper_dict)

        return pepper_dict
    


if __name__ == '__main__':
    PepperDetection = PepperFruitDetector(file_path='/dataset/peduncle',
                                          yolo_weight_path="../yolov8_scripts/weights/pepper_fruit_best_4.pt")
    PepperDetection.run_detection(img_path='/dataset/peduncle', show_result=False)
    print(PepperDetection)
