from typing import List

import ultralytics
from ultralytics import YOLO

from pepper_fruit import PepperFruit
from pepper_fruit_utils import print_pepperdetection, get_all_image_path_in_folder, read_image, print_result_boxes, remove_overlapping_boxes


class PepperFruitDetector:
    def __init__(self, file_path: str, yolo_weight_path: str, img=None):

        ultralytics.checks()

        self._model: YOLO = YOLO(yolo_weight_path)
        self._path: str = file_path
        self._classes: List[str] = ["pepper"]

        self._imgs_path: List[str] = list()
        self._img = img

    @property
    def classes(self):
        return self._classes

    def __str__(self):
        return print_pepperdetection(self)

    def run_detection(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.25):
        self._imgs_path = get_all_image_path_in_folder(self._path)
        return self.predict_pepper(img_path, show_result, print_result, thresh=thresh)
    
    def predict_pepper(self, img_path, show_result: bool = False, print_result: bool = False, thresh=0.25):
        pepper_list = dict()
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
                pepper_list[pepper_count] = pepper
                pepper_count += 1

        # UserWarning: Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure. plt.show()
        if show_result:
            for result in results:
                res_plotted = result.plot()
                # cv2.imshow("result", res_plotted)
        if print_result:
            print_result_boxes(pepper_list)

        pepper_list = remove_overlapping_boxes(pepper_list)

        return pepper_list
    
    def predict_peppers(self, show_result: bool = False, print_result: bool = False):

        for img_path in self._imgs_path:
            detected_frame = self.predict_pepper(img_path, show_result, print_result)


if __name__ == '__main__':
    PepperDetection = PepperFruitDetector(file_path='/dataset/peduncle',
                                          yolo_weight_path="../yolov8_scripts/weights/pepper_fruit_best_5.pt")
    PepperDetection.run_detection(img_path='/dataset/peduncle', show_result=False)
    print(PepperDetection)
