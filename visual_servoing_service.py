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
from pepper_ws.srv import visual_servo

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
    img = get_image()
    img_name=str(time.time()).split('.')[0]
    cv2.imwrite(os.getcwd()+'/visual_servoing/'+img_name+'.png', img)
    try:
        pp = PepperPeduncleDetector(os.getcwd()+'/visual_servoing/'+img_name+'.png', yolo_weight_path=os.getcwd()+"/weights/pepper_peduncle_best_3.pt")
        peduncle_list = pp.run_detection(os.getcwd()+'/visual_servoing/'+img_name+'.png')

        pepper_of_interest = None
        poi_xyz = (0, 0, 0)
        
        for k, v in peduncle_list.items():
            v.set_point_of_interaction(img.shape)
            (dx ,dy, dz) = get_xy_in_realworld(v.poi_px[0], v.poi_px[1]) #TODO

            if pepper_of_interest == None:
                pepper_of_interest = v
                poi_xyz = (dx ,dy, dz)

            elif int(dx) != 0 and abs(dx)<=abs(poi_xyz[0]):
                print(f"dx: {dx}, v.xyz: {poi_xyz}")
                pepper_of_interest = v
                poi_xyz = (dx ,dy, dz)
            else:
                continue

        if pepper_of_interest != None:
            pp.plot_results(peduncle_list, pepper_of_interest.poi_px, poi_xyz)
            got_depth = True
        print(colored(f"pepper of interest: \n{pepper_of_interest.poi_px}, {poi_xyz}"))
        return poi_xyz

    except Exception as e:
        print("Error in detecting pepper", e)
        return (0, 0, 0)
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
    # rospy.loginfo(peduncle_pose)
    # print(colored("published to topic", "yellow"))
    if z > 0.1:
        visual_servo_pub.publish(change_pose)

def handle_visual_servoing(req):
    global dx, dy, dz
    print(colored("Returning visual servoing", 'magenta'))
    if req.req_id == 0:
    # if req == 0:
        (dx ,dy, dz) = visual_servoing()
        print("before: ",(dx ,dy, dz) )
        (dx ,dy, dz) = (0.01-dx ,dy, dz)
        print(colored(f"in realsense world: has to move x, y, z {round(dx, 3), round(dy,3), round(dz,3)}",'blue'))
        if not got_depth:
            print(colored("sent 0 cuz failed", 'red'))
            return 0
        print(colored("sent 1 cuz success", 'blue'))
        return 1
    # else:
    #     continue


def vs_server():
    global dx
    rospy.init_node('visual_servoing_server')
    rate = rospy.Rate(10)
    rospack = rospkg.RosPack()
    os.chdir(rospack.get_path("pepper_ws"))
    s = rospy.Service('/perception/visual_servo', visual_servo, handle_visual_servoing)
    # handle_visual_servoing(0)
    while not rospy.is_shutdown():
        if dz > 0.1:
            start_time = time.time()
            # print("visual servo results: x, y, z", dx, dy, dz)
            while time.time() - start_time<30:
                publish_d(dx ,dy, dz)
        rospy.sleep(1)
    rospy.spin()

if __name__=="__main__":
    print(vs_server())