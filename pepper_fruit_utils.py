import collections
import math
import os
from typing import Dict

import cv2
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image
from shapely import Polygon

from pepper_fruit import PepperFruit
from pepper_peduncle import PepperPeduncle
from pepper_utils import *


def print_result_boxes(pepper_list):
    print(f"detected {len(pepper_list)} peppers!")
    for k, pepper in pepper_list.items():
        print(pepper)
        # draw_bounding_box(result.orig_img, box.cls, box.conf, x, y, x + w, y - h)


def print_result_masks(detected_img):
    print(f"detected {detected_img.pepper_peduncle_count} peduncles!")
    for peduncle in detected_img.pepper_peduncle_detections:
        print(peduncle)
        # draw_bounding_box(result.orig_img, box.cls, box.conf, x, y, x + w, y - h)


def draw_bounding_box(confidence, x, y, w, h, color="blue", fill=False):
    # Get the current reference
    ax = plt.gca()

    # plot the bounding box
    rect = patches.Rectangle((x - w / 2, y - h / 2), w, h, linewidth=2, edgecolor=color, facecolor='none')

    # Add the patch to the Axes
    ax.add_patch(rect)

    # plot the centroid of pepper
    # plt.plot(x, y, 'b*')

    # plot conf
    plt.text(x - w / 2, y - h / 2, round(float(confidence[0]), 2), color='red', fontsize=10)  # round(confidence[0], 2)


def draw_bounding_polygon(confidence, mask, img_shape, color="blue", fill=True):
    """
    Draw the bounding polygons associated with a peduncle.
    :param confidence: Confidence score associated with a mask.
    :param mask: Mask associated with a peduncle with values between 0 and 1.
    :param img_shape: The shape of the image with
    :return: None.
    """

    mask = torch.transpose(mask, 0, 1)
    polygon = Polygon(mask.T)
    x, y = polygon.exterior.xy
    x_scaled = [i * img_shape[1] for i in x]
    y_scaled = [i * img_shape[0] for i in y]
    if fill:
        plt.fill(x_scaled, y_scaled, color=color, alpha=0.7)
    else:
        plt.fill(x_scaled, y_scaled, color=color, alpha=1)
    plt.plot(*polygon.exterior.xy)
    # p = gpd.GeoSeries(polygon)


def draw_pepper(one_frame):
    img = np.asarray(Image.open(one_frame.img_path))
    img_name = one_frame.img_path.split('/')[-1].split('.')[0]
    plt.imshow(img)

    put_title(one_frame)
    for peduncle in one_frame.pepper_peduncle_detections.values():
        mask = peduncle.mask
        draw_bounding_polygon(peduncle.conf, mask, one_frame.img_shape, color='black', fill=False)
    for pepper_fruit in one_frame.pepper_fruit_detections.values():
        xywh = pepper_fruit.xywh
        x = int(xywh[0])
        y = int(xywh[1])
        w = int(xywh[2])
        h = int(xywh[3])
        draw_bounding_box(pepper_fruit.conf, x, y, w, h, color="black", fill=False)

    for idx, pepper in one_frame.pepper_detections.items():
        r = np.round(np.random.rand(), 1)
        g = np.round(np.random.rand(), 1)
        b = np.round(np.random.rand(), 1)
        # a = np.round(np.clip(np.random.rand(), 0, 1), 1)
        color = (r, g, b)
        pepper_fruit = pepper.pepper_fruit
        pepper_peduncle = pepper.pepper_peduncle
        xywh = pepper_fruit.xywh
        x = int(xywh[0])
        y = int(xywh[1])
        w = int(xywh[2])
        h = int(xywh[3])
        draw_bounding_box(pepper_fruit.conf, x, y, w, h, color=color)

        mask = pepper_peduncle.mask
        draw_bounding_polygon(pepper_peduncle.conf, mask, one_frame.img_shape, color=color)
    # plt.axis('off')
    plt.savefig(
        f"{os.getcwd()}/result/{img_name}_pepper_result.png", 
        bbox_inches='tight', pad_inches=1)
    plt.clf()
    plt.cla()


def draw_pepper_fruits(one_frame):
    img = np.asarray(Image.open(one_frame.img_path))
    img_name = one_frame.img_path.split('/')[-1].split('.')[0]
    plt.imshow(img)

    put_title(one_frame)
    for pepper in one_frame.pepper_fruit_detections.values():
        xywh = pepper.xywh
        x = int(xywh[0])
        y = int(xywh[1])
        w = int(xywh[2])
        h = int(xywh[3])
        draw_bounding_box(pepper.conf, x, y, w, h)

    plt.savefig(
        f"{os.getcwd()}/result/{img_name}_fruit_result.png")
    plt.clf()
    plt.cla()


def draw_pepper_peduncles(detected_frame):
    img = np.asarray(Image.open(detected_frame.img_path))
    img_name = detected_frame.img_path.split('/')[-1].split('.')[0]
    plt.imshow(img)
    put_title(detected_frame)
    for peduncle in detected_frame.pepper_peduncle_detections.values():
        mask = peduncle.mask
        draw_bounding_polygon(peduncle.conf, mask, detected_frame.img_shape)

    plt.savefig(f"{os.getcwd()}/result/{img_name}_peduncle_result.png")
    plt.clf()
    plt.cla()


def print_pepperdetection(pd):
    output = "\n============================================\n"
    output += f"PepperDetection Results:\n"
    output += f"folder_path={pd._path}\n# of detected_images: {len(pd.detected_frames)}\n"
    for detected_img in pd.detected_frames:
        output += f"\t{detected_img.img_path.split('/')[-1]}\n"
        output += f"\t\tDetected {detected_img.pepper_fruit_count} peppers\n"
        for pepper in detected_img.pepper_fruit_detections:
            output += f"\t\t\t{pepper}\n"
        output += f"\t\tDetected {detected_img.pepper_peduncle_count} peduncles\n"
        # for peduncle in detected_img.pepper_peduncle_detections:
        # output += f"\t\t\t{peduncle}\n"
    output += "============================================\n"
    return output




def is_above(pepper_fruit: PepperFruit, pepper_peduncle: PepperPeduncle):
    # print(pepper_fruit.xywh[1], pepper_peduncle.xywh[1])
    return pepper_fruit.xywh[1] >= pepper_peduncle.xywh[1]


def match_pepper_fruit_peduncle(pepper_fruit_detections: Dict[int, PepperFruit],
                                pepper_peduncle_detections: Dict[int, PepperPeduncle]):
    pepper_fruit_peduncle_distances = []
    for pepper_fruit in pepper_fruit_detections.values():
        min_dist = math.inf
        peduncle_match = None
        for pepper_peduncle in pepper_peduncle_detections.values():
            dist = distance_between_pepper_fruit_peduncle(pepper_fruit, pepper_peduncle)
            x, y, w, h = pepper_fruit.xywh
            box1 = [[x - w / 2, y - h / 2], [x + w / 2, y - h / 2], [x + w / 2, y + h / 2], [x - w / 2, y + h / 2]]
            x, y, w, h = pepper_peduncle.xywh
            box2 = [[x - w / 2, y - h / 2], [x + w / 2, y - h / 2], [x + w / 2, y + h / 2], [x - w / 2, y + h / 2]]
            iou = calculate_iou(box1, box2)
            if dist < min_dist and iou > 0:
                peduncle_match = pepper_peduncle
                min_dist = dist
        if not peduncle_match:
            peduncle_match = PepperPeduncle(-1)
        pepper_fruit_peduncle_distances.append(((pepper_fruit.number, peduncle_match.number), min_dist))

    pepper_fruit_peduncle_match = remove_duplicate_peduncles(pepper_fruit_peduncle_distances)
    return pepper_fruit_peduncle_match


def choose_unmatching(duplicate_list):
    # ((pf_number, pp_number), distance)
    pepper_delete = list()
    for duplicates in duplicate_list:
        duplicates = sorted(duplicates, key=lambda d: d[1])
        pepper_delete.append(duplicates[1:])
    return pepper_delete


def remove_duplicate_peduncles(pepper_fruit_peduncle_distances: list):
    # remove duplicate peduncles

    detetected_pepper_fruit = []
    detected_pepper_peduncle = []

    for (pf, pp), d in pepper_fruit_peduncle_distances:
        detetected_pepper_fruit.append(pf)
        detected_pepper_peduncle.append(pp)
    duplicate_pepper_fruit = [item for item, count in collections.Counter(detetected_pepper_fruit).items() if count > 1]
    duplicate_pepper_peduncle = [item for item, count in collections.Counter(detected_pepper_peduncle).items() if
                                 count > 1]

    pp_duplicate_list = list()
    for pepper_peduncle in duplicate_pepper_peduncle:  # index of peppers
        duplicate_list = list()
        for i in range(len(pepper_fruit_peduncle_distances)):
            (pf, pp), d = pepper_fruit_peduncle_distances[i]
            if pp == pepper_peduncle:
                duplicate_list.append(pepper_fruit_peduncle_distances[i])
        pp_duplicate_list.append(duplicate_list)
    pepper_peduncle_delete = choose_unmatching(pp_duplicate_list)
    for d in pepper_fruit_peduncle_distances:
        if d in pepper_peduncle_delete:
            pepper_fruit_peduncle_distances.remove(d)

    return pepper_fruit_peduncle_distances


def distance_between_pepper_fruit_peduncle(pepper_fruit: PepperFruit, pepper_peduncle: PepperPeduncle):
    pepper_fruit_xywh = pepper_fruit.xywh
    pepper_peduncle_xywh = pepper_peduncle.xywh

    pf_coords = [pepper_fruit_xywh[0].item(), pepper_fruit_xywh[1].item()]
    pp_coords = [pepper_peduncle_xywh[0].item(), pepper_peduncle_xywh[1].item()]
    distance = math.dist(pf_coords, pp_coords)
    return distance



def calculate_iou(box_1, box_2):
    '''
    :param box_1: (4 x 2) tl, tr, br, bl
    :param box_2: (4 x 2) tl, tr, br, bl
    :return: iou from 0-1
    '''
    poly_1 = Polygon(box_1)
    poly_2 = Polygon(box_2)

    if poly_1.intersects(poly_2): 
        iou = poly_1.intersection(poly_2).area / poly_1.union(poly_2).area
    else:
        iou = 0

    # print(f"iou: {iou}")
    return iou


if __name__ == '__main__':
    image = cv2.imread("/home/jy/PycharmProjects/Perception-Resources/dataset/colorful/bell-peppers-in-season.jpg")
    # cv2.imwrite("img1.jpg", image)
    # image = red_to_green(image)
    # cv2.imwrite("img2.jpg", image)
    red_to_green_2(image)
