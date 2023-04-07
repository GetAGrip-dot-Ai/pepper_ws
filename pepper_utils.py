import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
# from realsense_camera import *

def get_img_size(img_path):
    img = read_image(img_path)
    return img.shape


def get_all_image_path_in_folder(path):
    # print("I am at", os.getcwd())
    # print('I want ', os.getcwd()+path)
    img_list = list()
    for dirs, subdir, files in os.walk(os.getcwd()+path):
        for file_name in files:
            if file_name.endswith(".jpeg") or file_name.endswith(".jpg") or file_name.endswith(".png"):
                rgb_file = dirs + os.sep + file_name
                img_list.append(rgb_file)
    # print("all images in folder: ", img_list)
    return img_list[:]


def read_image(img_path):
    img = cv2.imread(img_path)
    img = np.asarray(img)
    return img


def put_title(detected_frame):
    # displaying the title
    plt.title(
        label=f"Pepper: {len(detected_frame.pepper_fruit_detections)} Peduncle: {len(detected_frame.pepper_peduncle_detections)}",
        fontsize=10,
        color="black")
    

def get_image_from_webcam():
    camera = cv2.VideoCapture(6)
    # camera = cv2.VideoCapture(0)
    # 4: dotted camera
    # 6: rgb camera
    while True:
        return_value, image = camera.read()
            # gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        image = cv2.flip(image, 1)  # <class 'numpy.ndarray'>
        cv2.imshow('image', image)
        k = cv2.waitKey(0)
        if k==27:
            print("hey")
            camera.release()
            cv2.destroyAllWindows()
            return image
            # break
    camera.release()
    cv2.destroyAllWindows()
    return image


def red_to_green(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_lo = np.array([50, 0, 0])
    red_hi = np.array([255, 255, 255])
    # Mask image to only select browns
    mask = cv2.inRange(hsv, red_lo, red_hi)
    cv2.imwrite("mask.jpg", mask)
    # img[mask > 0] = img[mask > 0] * [0.3, 0, 0] + [0, 130, 0]
    img[mask > 0] = 0
    return img


def red_to_green_2(img):
    b, g, r = cv2.split(img)  # get b,g,r
    rgb_img = cv2.merge([r, g, b])
    # plt.imshow(rgb_img)

    x, y, z = np.shape(img)
    red = np.zeros((x, y, z), dtype=int)
    green = np.zeros((x, y, z), dtype=int)
    blue = np.zeros((x, y, z), dtype=int)
    for i in range(0, x):
        for j in range(0, y):
            red[i][j][0] = rgb_img[i][j][0]
            green[i][j][1] = rgb_img[i][j][1]
            blue[i][j][2] = rgb_img[i][j][2]
    # plt.imshow(red)
    # # plt.show()
    # plt.imshow(green)
    # # plt.show()
    # plt.imshow(blue)
    # plt.show()

    retrack_original = np.zeros((x, y, z), dtype=int)
    for i in range(0, x):
        for j in range(0, y):
            retrack_original[i][j][0] = red[i][j][0] * 0.2 // 1
            retrack_original[i][j][1] = green[i][j][1]
            retrack_original[i][j][2] = blue[i][j][2]
    # cv2.imwrite('ori.jpg', retrack_original)
    plt.imshow(retrack_original)
    plt.show()
    return retrack_original

if __name__=="__main__":
    img = get_image_from_webcam()
    cv2.imwrite('he.png', img)
     # plt.imshow(img)
    # plt.imsave(img, "hi.png")

    # First import the library
    # import pyrealsense2 as rs

    # # Create a context object. This object owns the handles to all connected realsense devices
    # pipeline = rs.pipeline()
    # pipeline.start()

    # try:
    #     while True:
    #         # Create a pipeline object. This object configures the streaming camera and owns it's handle
    #         frames = pipeline.wait_for_frames()
    #         depth = frames.get_depth_frame()
    #         if not depth: continue

    #         # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
    #         coverage = [0]*64
    #         for y in range(480):
    #             for x in range(640):
    #                 dist = depth.get_distance(x, y)
    #                 if 0 < dist and dist < 1:
    #                     coverage[x//10] += 1

    #             if y%20 == 19:
    #                 line = ""
    #                 for c in coverage:
    #                     line += " .:nhBXWW"[c//25]
    #                 coverage = [0]*64
    #                 print(line)

    # finally:
    #     pipeline.stop()
