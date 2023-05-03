import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os
import time
import matplotlib.pyplot as plt
from termcolor import colored


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Functions that get an image and depth from the realsense camera
"""


def get_depth(realsense_camera, x=320, y=240):
    # print(f"x: {x}, y:{y}")
    x, y = y, x
    dx ,dy, dz = -1, -1, -1

    # Configure depth and color streams
    pipeline = realsense_camera.pipeline
    colorizer = realsense_camera.colorizer
    align = realsense_camera.align

    print(colored("Getting depth", "blue"))
    try:
        distance = 0
        count = 0
        file_name = str(time.time()).split('.')[0]
        path = os.getcwd() + '/depthlog/'
        isExist = os.path.exists(path)
        if not isExist:
            os.makedirs(path)

        while count < 10:
            count += 1
            frames = pipeline.wait_for_frames()
            aligned_frames =  align.process(frames)
            color_frame_not_aligned = frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth = depth_frame.get_distance(int(x), int(y))
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            # dx -= - 0.0325 
            color_image = np.asanyarray(color_frame.get_data())
            color_frame_not_aligned = np.asanyarray(color_frame_not_aligned.get_data())

            depth_colormap = np.asanyarray(
                        colorizer.colorize(depth_frame).get_data())
            img = np.hstack((color_image, depth_colormap))

        return dx ,dy, dz
    
    except Exception as e:
        print(colored(f"Problem in realsense_utils.py->get_depth! {e}", "red"))
        # pipeline.stop()
        return dx ,dy, dz
    
def get_depth_orig(x=320, y=240):
    # print(f"x: {x}, y:{y}")
    x, y = y, x
    x, y = int(x), int(y)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        print(colored("NO IMAGE READ BY THE RGBD CAMERA", "red"))
        return

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    # align_to = rs.stream.depth
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        count = 0

        while count <100:
            count += 1
            frames = pipeline.wait_for_frames()
            aligned_frames =  align.process(frames)
            color_frame_not_aligned = frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth = depth_frame.get_distance(x, y)
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            color_frame_not_aligned = np.asanyarray(color_frame_not_aligned.get_data())
        pipeline.stop()
        print("in realsesne_utilass.py: ", dx, dy, dz)
        return dx ,dy, dz

    
    except Exception as e:
        print(colored(f"Problem in realsense_utils.py->get_depth_orig! {e}", "red"))
        pipeline.stop()
        return dx ,dy, dz
    
def get_image(realsense_camera):
    # Configure depth and color streams
    pipeline = realsense_camera.pipeline

    print(colored("Getting image", "blue"))
    try:
        count  = 0
        while count < 10 :
            count += 1

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

        return color_image

    finally:
        # Stop streaming
        # pipeline.stop()
        print(colored("Got image", "blue"))


def get_image_orig():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False

    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        return None

    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    print(colored("Camera started: getting image", "blue"))
    try:
        count  = 0
        while count < 30 :
            count += 1

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

        return color_image

    finally:
        # Stop streaming
        pipeline.stop()
        print(colored("Got image", "blue"))
# THINGS TO CHANGE
'''
- need to check how the rate is at for the kinova
'''
