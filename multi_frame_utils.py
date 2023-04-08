import numpy as np
from pepper_fruit_utils import *


def get_best_fruit_match(fruit1, fruits2):
    x, y, w, h = fruit1.xywh
    box1 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]

    iou_list = []

    for fruit2 in fruits2:
        x, y, w, h = fruit2.xywh
        box2 = [[x - w/2, y - h/2], [x + w/2, y - h/2], [x + w/2, y + h/2], [x - w/2, y + h/2]]

        iou = calculate_iou(box1, box2)
        iou_list.append((fruit2, iou))

    iou_list.sort(key=lambda x: x[1], reverse=True)

    return iou_list[0]


def update_fruit_occurences(fruits_frame1, fruits_frame2):
    for fruit1 in fruits_frame1:
        fruit2, max_iou = get_best_fruit_match(fruit1, fruits_frame2)
        print(f"fruit 2: {fruit2.number}")

        if max_iou > 0.3:  # Need to tune
            fruit1.occurences += 1
            fruit2.occurences += 1
            print(f"fruit 1: {fruit1.number}, fruit 2: {fruit2.number}")


def update_true_positives(fruits, max_frames):
    for fruit in fruits:
        if fruit.occurences >= 0.5*max_frames:
            fruit.true_positive = True


def get_fruits(frame1_number, frame2_number, fruits_frame1, fruits_frame2):
    if frame1_number != frame2_number:
        fruits_list = []

        for fruit1 in fruits_frame1:
            if fruit1.true_positive:
                _, max_iou = get_best_fruit_match(fruit1, fruits_frame2)
    
                if max_iou < 0.3:  # Need to tune
                    fruits_list.append(fruit1)

        return fruits_list
    else:
        fruits_list = []

        for fruit1 in fruits_frame1:
            if fruit1.true_positive:
                fruits_list.append(fruit1)

        return fruits_list