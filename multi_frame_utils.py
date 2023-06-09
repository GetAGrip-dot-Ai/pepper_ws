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


def update_fruit_occurences(fruits_frame1, fruits_frame2, frame1_number, frame2_number):
    for fruit1 in fruits_frame1:
        fruit2, max_iou = get_best_fruit_match(fruit1, fruits_frame2)

        if max_iou > 0.3:  # Need to tune
            fruit1.occurences += 1
            fruit2.occurences += 1

            fruit1.add_associated_pepper(frame2_number, fruit2)
            fruit2.add_associated_pepper(frame1_number, fruit1)
            print(f"fruit 1: {fruit1.number}, fruit 2: {fruit2.number}")


def update_true_positives(fruits, max_frames):
    for fruit in fruits:
        if fruit.occurences >= 0.5*max_frames:
            fruit.true_positive = True
    

def get_all_fruits(frames):
    all_frames_positive_fruits = []
    unique_positive_fruits = {}

    for i in reversed(range(len(frames))):
        frame = frames[i]
        
        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                all_frames_positive_fruits.append(fruit)

    for i in reversed(range(len(frames))):
        frame = frames[i]

        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                if fruit in all_frames_positive_fruits:
                    all_frames_positive_fruits.remove(fruit)

                    if frame.frame_number not in unique_positive_fruits:
                        unique_positive_fruits[frame.frame_number] = [fruit]
                    else:  
                        unique_positive_fruits[frame.frame_number].append(fruit)

                    for _, associated_fruit in fruit.associated_peppers:
                        if associated_fruit in all_frames_positive_fruits:
                            all_frames_positive_fruits.remove(associated_fruit)
            
    return unique_positive_fruits