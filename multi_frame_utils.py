import numpy as np
from pepper_fruit_utils import *
from pepper_peduncle_utils import *


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

    return iou_list[0] if len(iou_list) > 0 else (None, 0) 


def update_fruit_occurences(fruits_frame1, fruits_frame2, frame1_number, frame2_number):
    for fruit1 in fruits_frame1:
        fruit2, max_iou = get_best_fruit_match(fruit1, fruits_frame2)

        if max_iou > 0.3:  # Need to tune
            fruit1.occurences += 1
            fruit2.occurences += 1

            fruit1.add_associated_fruit(frame2_number, fruit2.number)
            fruit2.add_associated_fruit(frame1_number, fruit1.number)
            # print(f"fruit 1: {fruit1.number}, fruit 2: {fruit2.number}")


def update_fruit_true_positives(fruits, max_frames):
    for fruit in fruits:
        if fruit.occurences >= 0.5*max_frames:
            fruit.true_positive = True
    

def get_all_fruits(frames):
    all_frames_positive_fruits = []
    unique_positive_fruits = {}
    unmatched_positive_fruits = {}

    for i in range(len(frames)):
        frame = frames[i]
        
        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                all_frames_positive_fruits.append(fruit)

    # print(all_frames_positive_fruits)

    for i in reversed(range(len(frames))):
        frame = frames[i]
        # print(i)

        for fruit in frame.pepper_fruit_detections.values():
            if fruit.true_positive:
                if fruit in all_frames_positive_fruits:

                    all_frames_positive_fruits.remove(fruit)
                    # print(all_frames_positive_fruits)
                    
                    if fruit.parent_pepper != None:
                        if frame.frame_number not in unique_positive_fruits:
                            unique_positive_fruits[frame.frame_number] = [fruit]
                        else:  
                            unique_positive_fruits[frame.frame_number].append(fruit)

                        for frame_number, associated_fruit_number in fruit.associated_fruits:
                            associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                            if associated_fruit in all_frames_positive_fruits:
                                all_frames_positive_fruits.remove(associated_fruit)
                    else:
                        found = False
                        while not found:
                            for frame_number, associated_fruit_number in fruit.associated_fruits:
                                
                                associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                                if associated_fruit.parent_pepper != None:
                                    if frame_number not in unique_positive_fruits:
                                        unique_positive_fruits[frame_number] = [associated_fruit]
                                    else:  
                                        unique_positive_fruits[frame_number].append(associated_fruit)

                                    found = True
                                    break
                            
                            if not found:
                                if frame.frame_number not in unmatched_positive_fruits:
                                    unmatched_positive_fruits[frame.frame_number] = [fruit]
                                else:  
                                    unmatched_positive_fruits[frame.frame_number].append(fruit)

                                found = True
                                break

                        for frame_number, associated_fruit_number in fruit.associated_fruits:
                            associated_fruit = frames[frame_number].pepper_fruit_detections[associated_fruit_number]

                            if associated_fruit in all_frames_positive_fruits:
                                all_frames_positive_fruits.remove(associated_fruit)

    return unique_positive_fruits, unmatched_positive_fruits


def get_best_peduncle_match(peduncle1, peduncles2):
    iou_list = []

    for peduncle2 in peduncles2:
        iou = calculate_peduncle_iou(peduncle1.mask, peduncle2.mask)
        iou_list.append((peduncle2, iou))

    iou_list.sort(key=lambda x: x[1], reverse=True)

    return iou_list[0] if len(iou_list) > 0 else (None, 0) 


def update_peduncle_occurences(peduncles_frame1, peduncles_frame2, frame1_number, frame2_number):
    for peduncle1 in peduncles_frame1:
        peduncle2, max_iou = get_best_peduncle_match(peduncle1, peduncles_frame2)

        if max_iou > 0.3:  # Need to tune
            peduncle1.occurences += 1
            peduncle2.occurences += 1

            peduncle1.add_associated_peduncle(frame2_number, peduncle2.number)
            peduncle2.add_associated_peduncle(frame1_number, peduncle1.number)
            # print(f"peduncle 1: {peduncle1.number}, peduncle 2: {peduncle2.number}")


def update_peduncle_true_positives(peduncles, max_frames):
    for peduncle in peduncles:
        if peduncle.occurences >= 0.5*max_frames:
            peduncle.true_positive = True


def get_all_peduncles(frames):
    all_frames_positive_peduncles = []
    unique_positive_peduncles = {}
    unmatched_positive_peduncles = {}

    for i in range(len(frames)):
        frame = frames[i]
        
        for peduncle in frame.pepper_peduncle_detections.values():
            if peduncle.true_positive:
                all_frames_positive_peduncles.append(peduncle)

    # print(all_frames_positive_peduncles)

    for i in reversed(range(len(frames))):
        frame = frames[i]

        for peduncle in frame.pepper_peduncle_detections.values():
            if peduncle.true_positive:
                if peduncle in all_frames_positive_peduncles:

                    all_frames_positive_peduncles.remove(peduncle)
                    # print(all_frames_positive_peduncles)
                    
                    if peduncle.parent_pepper != None:
                        if frame.frame_number not in unique_positive_peduncles:
                            unique_positive_peduncles[frame.frame_number] = [peduncle]
                        else:  
                            unique_positive_peduncles[frame.frame_number].append(peduncle)

                        for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                            associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                            if associated_peduncle in all_frames_positive_peduncles:
                                all_frames_positive_peduncles.remove(associated_peduncle)
                    else:
                        found = False
                        while not found:
                            for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                                
                                associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                                if associated_peduncle.parent_pepper != None:
                                    if frame_number not in unique_positive_peduncles:
                                        unique_positive_peduncles[frame_number] = [associated_peduncle]
                                    else:  
                                        unique_positive_peduncles[frame_number].append(associated_peduncle)

                                    found = True
                                    break
                            
                            if not found:
                                if frame.frame_number not in unmatched_positive_peduncles:
                                    unmatched_positive_peduncles[frame.frame_number] = [peduncle]
                                else:  
                                    unmatched_positive_peduncles[frame.frame_number].append(peduncle)

                                found = True
                                break

                        for frame_number, associated_peduncle_number in peduncle.associated_peduncles:
                            associated_peduncle = frames[frame_number].pepper_peduncle_detections[associated_peduncle_number]

                            if associated_peduncle in all_frames_positive_peduncles:
                                all_frames_positive_peduncles.remove(associated_peduncle)
                
    return unique_positive_peduncles, unmatched_positive_peduncles


def get_image_webcam():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    return frame