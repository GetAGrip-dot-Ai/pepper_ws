import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import os
import cv2
import numpy as np
from PIL import Image, ImageDraw
from shapely import Polygon
from scipy.optimize import curve_fit
from skimage.morphology import medial_axis
# import pepper_fruit_utils

from curve import Curve


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Functions that support the calculation of a POI from YOLOv8 outputs
"""


def parabola(t, a, b, c):
    # print("params", params)
    return a * t ** 2 + b * t + c


def get_segment_from_mask(mask, img_shape):

    points = []
    for i in range(len(mask)):
        points.append(img_shape[1]*mask[i, 0])
        points.append(img_shape[0]*mask[i, 1])

    img = Image.new('L', (img_shape[1], img_shape[0]), 0)
    ImageDraw.Draw(img).polygon(points, outline=1, fill=1)
    segment = np.array(img)

    return segment


def fit_curve_to_mask(mask, img_shape, pepper_fruit_xywh, pepper_peduncle_xywh):

    curve = Curve()
    segment = get_segment_from_mask(mask, img_shape)

    medial_img, _ = medial_axis(segment, return_distance=True)

    x, y = np.where(medial_img == 1)
    
    params1, _ = curve_fit(parabola, y, x)
    a, b, c = params1
    fit_curve_x = parabola(y, a, b, c)
    
    params2, _ = curve_fit(parabola, x, y)
    a, b, c = params2
    fit_curve_y = parabola(x, a, b, c)

    if np.linalg.norm(x - fit_curve_x) < np.linalg.norm(y - fit_curve_y):
        curve.parabola_direction = 'vertical'
        curve.params = params1

        # xywh: x: along columns, y: along rows
        if pepper_fruit_xywh[0] < pepper_peduncle_xywh[0]:
            # Sorted assuming that the pepper to the left of the peduncle
            # Sorting with respect to y in ascending order
            sorted_x = np.array([x for _, x in sorted(zip(y, x))])
            sorted_y = np.array([y for y, _ in sorted(zip(y, x))])
        else:
            # Sorted assuming that the pepper to the right of the peduncle
            # Sorting with respect to y in descending order
            sorted_x = np.flip(np.array([x for _, x in sorted(zip(y, x))]))
            sorted_y = np.flip(np.array([y for y, _ in sorted(zip(y, x))]))

        curve.curve_y = sorted_y
        curve.curve_x = parabola(sorted_y, curve.params[0], curve.params[1], curve.params[2])
    else:
        curve.parabola_direction = 'horizontal'
        curve.params = params2

        if pepper_fruit_xywh[1] < pepper_peduncle_xywh[1]:
            # Sorted assuming that the pepper is above the peduncle
            # Sorted with respect to x in ascending order
            sorted_x = np.array([x for x, _ in sorted(zip(x, y))])
            sorted_y = np.array([y for _, y in sorted(zip(x, y))])
        else:
            # Sorted assuming that the pepper is below the peduncle
            # Sorted with respect to x in descending order
            sorted_x = np.flip(np.array([x for x, _ in sorted(zip(x, y))]))
            sorted_y = np.flip(np.array([y for _, y in sorted(zip(x, y))]))

        curve.curve_x = sorted_x
        curve.curve_y = parabola(sorted_x, curve.params[0], curve.params[1], curve.params[2])

    return curve


def determine_poi(curve, percentage, total_curve_length):
    for idx in range(len(curve.curve_y)):
        curve_length = curve.curve_length(idx)
        if abs(curve_length - percentage * total_curve_length) < 2:
            return curve.curve_x[idx], curve.curve_y[idx]
    
    return curve.curve_x[len(curve.curve_y) // 2], curve.curve_y[len(curve.curve_y) // 2]


def determine_next_point(curve, poi, pepper_fruit_xywh, pepper_peduncle_xywh):
    if curve.parabola_direction == 'vertical':
        # xywh: x: along columns, y: along rows
        if pepper_fruit_xywh[0] < pepper_peduncle_xywh[0]:
            # Assuming that the pepper to the left of the peduncle
            point_y = poi[1] + 1
            point_x = parabola(point_y, curve.params)
        else:
            # Assuming that the pepper to the right of the peduncle
            point_y = poi[1] - 1
            point_x = parabola(point_y, curve.params)
    else:
        if pepper_fruit_xywh[1] < pepper_peduncle_xywh[1]:
            # Assuming that the pepper is above the peduncle
            point_x = poi[0] + 1
            point_y = parabola(point_x, curve.params)
        else:
            # Assuming that the pepper is below the peduncle
            point_x = poi[0] - 1
            point_y = parabola(point_x, curve.params)

    return point_x, point_y


def calculate_peduncle_iou(mask1, mask2):
    poly_1 = Polygon(mask1)
    poly_2 = Polygon(mask2)

    iou = poly_1.intersection(poly_2).area / poly_1.union(poly_2).area if poly_1.intersects(poly_2) else 0
    return iou


def draw_poi(one_frame):
    img = cv2.imread(one_frame.img_path, 0)
    img_name = one_frame.img_path.split('/')[-1].split('.')[0]
    plt.imshow(img)

    for pepper in one_frame.pepper_detections.values():
        poi = pepper.pepper_peduncle.poi
        plt.plot(poi[1]*100, poi[0]*100, 'ro', markersize=2)

    plt.axis('off')
    plt.savefig(
        f"{os.getcwd()}/result/{img_name}_poi_result.png",
        bbox_inches='tight', pad_inches=1)
    plt.clf()
    plt.cla()


def draw_poi(one_frame):
    img = cv2.imread(one_frame.img_path, 0)
    img_name = one_frame.img_path.split('/')[-1].split('.')[0]
    plt.imshow(img)

    for pepper in one_frame.pepper_detections.values():
        poi = pepper.pepper_peduncle.poi
        plt.plot(poi[1]*100, poi[0]*100, 'ro', markersize=2)

    plt.axis('off')
    plt.savefig(
        f"{os.getcwd()}/result/{img_name}_poi_result.png",
        bbox_inches='tight', pad_inches=1)
    plt.clf()
    plt.cla()


def draw_one_poi(pepper_peduncle):
    poi = pepper_peduncle.poi
    plt.plot(poi[1]*100, poi[0]*100, 'ro', markersize=2)

