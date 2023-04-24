#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
from pepper_peduncle_detector import PepperPeduncleDetector
from realsense_utils import get_image
from geometry_msgs.msg import Pose
import rospy, rospkg
from termcolor import colored
from pepper_peduncle_utils import draw_one_poi
# from pepper_ws.srv import visual_servo
from realsense_utils import *

dx, dy, dz = 0, 0, 0
got_depth = False

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
        while count < 100:
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

            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            depth = depth_frame.get_distance(x, y)
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            # dx -= - 0.0325
            # k = cv2.waitKey(0)
            # if k==27:
            #     cv2.destroyAllWindows()
            #     return (dx ,dy, dz)

    finally:
        # Stop streaming
        pipeline.stop()
        print(colored(f"x, y, z {round(dx, 3), round(dy,3), round(dz,3)}",'red'))

    return (dx ,dy, dz)

def visual_servoing():

    global got_depth
    start_time = time.time()
    img = get_image()
    print(colored(f"Get img took {time.time()-start_time}", 'cyan'))

    img_name=str(time.time()).split('.')[0]
    img_path = os.getcwd()+'/visual_servoing/'+img_name+'.png'

    cv2.imwrite(img_path, img)

    pepper_of_interest = None
    pepper_of_interest_poi_xyz = (0, 0, 0)

    try:
        pp = PepperPeduncleDetector(yolo_weight_path=os.getcwd()+"/weights/pepper_peduncle_best_4.pt")

        peduncle_list = pp.predict_peduncle(img_path)

        
        for peduncle_num, peduncle in peduncle_list.items():

            peduncle.set_point_of_interaction(img.shape)

            # (dx ,dy, dz) = get_xy_in_realworld(peduncle.poi_px[0], peduncle.poi_px[1]) #TODO
            start_time = time.time()
            (dx, dy, dz) = get_depth(peduncle.poi_px[0], peduncle.poi_px[1])
            print(colored(f"Get depth took {time.time()-start_time}", 'cyan'))
            if pepper_of_interest == None:
                print(colored("first pepper", 'magenta'))
                pepper_of_interest = peduncle
                pepper_of_interest_poi_xyz = (dx ,dy, dz)

            elif int(dx) != 0 and abs(dx)<=abs(pepper_of_interest_poi_xyz[0]):
                print(colored(f"changing pepper because [{dx}] is smaller than {pepper_of_interest_poi_xyz[0]}", 'magenta'))
                pepper_of_interest = v
                pepper_of_interest_poi_xyz = (dx ,dy, dz)
            else:
                print(colored(f"not changing pepper because [{dx}] is bigger than {pepper_of_interest_poi_xyz[0]}", 'magenta'))

        if pepper_of_interest != None:
            pp.plot_results(peduncle_list, pepper_of_interest.poi_px, pepper_of_interest_poi_xyz)
            got_depth = True

        print(colored(f"pepper of interest: \n, {pepper_of_interest_poi_xyz}", "blue"))
        return pepper_of_interest_poi_xyz

    except Exception as e:
        print(colored(f"Error in detecting pepper: {e}", "red"))
        return pepper_of_interest_poi_xyz
    # get the x, y, z in the realsense axis frame
    # this should be 0, offset of the camera in th rs frame's -z axis 
    # and the z is just not going to work because it dies at 0.15 depth

def publish_d(x, y, z):

    visual_servo_pub = rospy.Publisher('/perception/peduncle/dpoi', Pose, queue_size=10)
    change_pose = Pose()
    # change to the base_link frame 
    change_pose.position.x = float(z)
    change_pose.position.y = -float(x)
    change_pose.position.z = -float(y)
    change_pose.orientation.x = 0
    change_pose.orientation.y = 0
    change_pose.orientation.z = 0
    change_pose.orientation.w = 1
    if z > 0.1:
        visual_servo_pub.publish(change_pose)

def handle_visual_servoing(req):

    global dx, dy, dz

    print(colored("Starting visual servoing", 'blue'))
    # if req.req_id == 0:
    if req == 0:
        (dx ,dy, dz) = visual_servoing()
        # print("before: ",(dx ,dy, dz) )

        (dx ,dy, dz) = (dx-0.03 ,-(dy+0.04), dz) # parameter tuning

        print(colored(f"in realsense world: has to move x, y, z {round(dx, 3), round(dy,3), round(dz,3)}",'blue'))


        if dz<=0.2:
            print(colored("Visual servoing failed :( Returning 0", 'red'))
            return 0
        else:
            print(colored("Visual servoing success :) Returning 1", 'green'))
            return 1
    # else:
    #     continue


def vs_server():
    global dx
    rospy.init_node('visual_servoing_server')

    rospack = rospkg.RosPack()
    os.chdir(rospack.get_path("pepper_ws"))

    # s = rospy.Service('/perception/visual_servo', visual_servo, handle_visual_servoing)
    handle_visual_servoing(0)

    while not rospy.is_shutdown():
        if get_depth:
            start_time = time.time()
            print(colored(f"Visual servo results: x:{dx} y:{dy} z:{dz}", "green"))
            while time.time() - start_time<1:
                publish_d(dx ,dy, dz)
    # rospy.spin()

if __name__=="__main__":
    print(vs_server())