import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
import matplotlib.pyplot as plt
from pepper_peduncle_detector import PepperPeduncleDetector
from realsense_utils import get_image
def get_xy_in_realworld(x=350, y=200):
    y, x = int(x), int(y)
    pipeline = rs.pipeline()
    config = rs.config()

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
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    count = 0
    try:
        while count < 10:
            count += 1
            frames = pipeline.wait_for_frames()
            aligned_frames =  align.process(frames)
            color_frame_not_aligned = frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            color_frame_not_aligned = np.asanyarray(color_frame_not_aligned.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.circle(images, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow('RealSense', images)

            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth = depth_frame.get_distance(x, y)
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            dx -= - 0.0325
            print("x, y, z", round(dx, 3), round(dy,3), round(dz,3))
            
            k = cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows()
                return (dx ,dy, dz)

    finally:
        # Stop streaming
        pipeline.stop()

    return (dx ,dy, dz)

def visual_servoing():
    img = get_image()
    img_name=str(time.time()).split('.')[0]
    cv2.imwrite(os.getcwd()+'/visual_servoing/'+img_name+'.png', img)
    try:
        pp = PepperPeduncleDetector(os.getcwd()+'/visual_servoing/'+img_name+'.png', yolo_weight_path="weights/pepper_peduncle_best_2.pt")
        peduncle_list = pp.run_detection(os.getcwd()+'/visual_servoing/'+img_name+'.png')
        for k, v in peduncle_list.items():
            v.set_point_of_interaction(img.shape)
    except Exception as e:
        print("Error in detecting pepper", e)
    # get the x, y, z in the realsense axis frame
    # this should be 0, offset of the camera in th rs frame's -z axis 
    # and the z is just not going to work because it dies at 0.15 depth
    (dx ,dy, dz) = get_xy_in_realworld(v.poi_px[0], v.poi_px[1])
    print("x, y, z", dx, dy, dz)
    retrun (dx ,dy, dz)

if __name__=="__main__":

    print(visual_servoing())