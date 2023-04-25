import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os
import time
import matplotlib.pyplot as plt
from termcolor import colored

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

        # plt.imshow(img)
        # plt.axis('on')
        # plt.plot(x, y,  'r*', markersize=5)
        # plt.plot(x+640, y, 'b*', markersize=5)
        # plt.plot(0, 0, 'g*', markersize=50)
        # plt.savefig(path+file_name+str(count)+'.png')
        # plt.cla()
        # plt.clf()
        # plt.close()
        # print(colored("Depth image saved to : ", path+file_name+str(count)+'.png', "blue"))
            
        # pipeline.stop()

        return dx ,dy, dz
    
    except Exception as e:
        print(colored(f"Problem in realsense_utils.py->get_depth! {e}", "red"))
        # pipeline.stop()
        return dx ,dy, dz
    
def get_depth_orig(x=320, y=240):
    # print(f"x: {x}, y:{y}")
    x, y = y, x
    dx ,dy, dz = -1, -1, -1

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    colorizer = rs.colorizer()


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
        print(colored("The demo requires Depth camera with Color sensor\nNO IMAGE READ BY THE RGBD CAMERA", "red"))
        return

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        distance = 0
        count = 0
        file_name = str(time.time()).split('.')[0]
        path = os.getcwd() + '/depthlog/'
        isExist = os.path.exists(path)
        if not isExist:
            os.makedirs(path)

        while count <30:
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
            # dx -= - 0.0325 
            color_image = np.asanyarray(color_frame.get_data())
            color_frame_not_aligned = np.asanyarray(color_frame_not_aligned.get_data())

            depth_colormap = np.asanyarray(
                        colorizer.colorize(depth_frame).get_data())
            # img = np.hstack((color_image, depth_colormap))

        # plt.imshow(img)
        # plt.axis('on')
        # plt.plot(x, y,  'r*', markersize=5)
        # plt.plot(x+640, y, 'b*', markersize=5)
        # plt.plot(0, 0, 'g*', markersize=50)
        # plt.savefig(path+file_name+str(count)+'.png')
        # plt.cla()
        # plt.clf()
        # plt.close()
        # print(colored("Depth image saved to : ", path+file_name+str(count)+'.png', "blue"))
            
        pipeline.stop()

        return dx ,dy, dz
    
    except Exception as e:
        print(colored(f"Problem in realsense_utils.py->get_depth! {e}", "red"))
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

            # If depth and color resolutions are different, resize color image to match depth image for display

            # # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            
            # k = cv2.waitKey(0)
            # if k==27:
            #     print("hey")
            #     cv2.destroyAllWindows()
            #     print("images", images.shape)
            #     return color_image
            #     # break

        # cv2.destroyAllWindows()

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

            # If depth and color resolutions are different, resize color image to match depth image for display

            # # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', images)
            
            # k = cv2.waitKey(0)
            # if k==27:
            #     print("hey")
            #     cv2.destroyAllWindows()
            #     print("images", images.shape)
            #     return color_image
            #     # break

        # cv2.destroyAllWindows()

        return color_image

    finally:
        # Stop streaming
        pipeline.stop()
        print(colored("Got image", "blue"))
# THINGS TO CHANGE
'''
- need to check how the rate is at for the kinova
'''
