import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os
import time
import matplotlib.pyplot as plt
from termcolor import colored
from RealsenseVisualizer import AppState

"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Functions that get an image and depth from the realsense camera
"""




def visualize_color_point_cloud(realsense_camera, poi):

    def project(v):
        """project 3d vector array to 2d"""
        h, w = out.shape[:2]
        view_aspect = float(h)/w

        # ignore divide by zero for invalid depth
        with np.errstate(divide='ignore', invalid='ignore'):
            proj = v[:, :-1] / v[:, -1, np.newaxis] * \
                (w*view_aspect, h) + (w/2.0, h/2.0)

        # near clipping
        znear = 0.03
        proj[v[:, 2] < znear] = np.nan
        return proj


    def view(v):
        """apply view transformation on vector array"""
        return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


    def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
        """draw a 3d line from pt1 to pt2"""
        p0 = project(pt1.reshape(-1, 3))[0]
        p1 = project(pt2.reshape(-1, 3))[0]
        if np.isnan(p0).any() or np.isnan(p1).any():
            return
        p0 = tuple(p0.astype(int))
        p1 = tuple(p1.astype(int))
        rect = (0, 0, out.shape[1], out.shape[0])
        inside, p0, p1 = cv2.clipLine(rect, p0, p1)
        if inside:
            cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


    def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
        """draw a grid on xz plane"""
        pos = np.array(pos)
        s = size / float(n)
        s2 = 0.5 * size
        for i in range(0, n+1):
            x = -s2 + i*s
            line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
                view(pos + np.dot((x, 0, s2), rotation)), color)
        for i in range(0, n+1):
            z = -s2 + i*s
            line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
                view(pos + np.dot((s2, 0, z), rotation)), color)


    def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
        """draw 3d axes"""
        line3d(out, pos, pos +
            np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
        line3d(out, pos, pos +
            np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
        line3d(out, pos, pos +
            np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


    def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
        """draw camera's frustum"""
        orig = view([0, 0, 0])
        w, h = intrinsics.width, intrinsics.height

        for d in range(1, 6, 2):
            def get_point(x, y):
                p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
                line3d(out, orig, view(p), color)
                return p

            top_left = get_point(0, 0)
            top_right = get_point(w, 0)
            bottom_right = get_point(w, h)
            bottom_left = get_point(0, h)

            line3d(out, view(top_left), view(top_right), color)
            line3d(out, view(top_right), view(bottom_right), color)
            line3d(out, view(bottom_right), view(bottom_left), color)
            line3d(out, view(bottom_left), view(top_left), color)


    def pointcloud(out, verts, texcoords, color, painter=True):
        """draw point cloud with optional painter's algorithm"""
        if painter:
            # Painter's algo, sort points from back to front

            # get reverse sorted indices by z (in view-space)
            # https://gist.github.com/stevenvo/e3dad127598842459b68
            v = view(verts)
            s = v[:, 2].argsort()[::-1]
            proj = project(v[s])
        else:
            proj = project(view(verts))

        if state.scale:
            proj *= 0.5**state.decimate

        h, w = out.shape[:2]

        # proj now contains 2d image coordinates
        j, i = proj.astype(np.uint32).T

        # create a mask to ignore out-of-bound indices
        im = (i >= 0) & (i < h)
        jm = (j >= 0) & (j < w)
        m = im & jm

        cw, ch = color.shape[:2][::-1]
        if painter:
            # sort texcoord with same indices as above
            # texcoords are [0..1] and relative to top-left pixel corner,
            # multiply by size and add 0.5 to center
            v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
        else:
            v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
        # clip texcoords to image
        np.clip(u, 0, ch-1, out=u)
        np.clip(v, 0, cw-1, out=v)

        # perform uv-mapping
        out[i[m], j[m]] = color[u[m], v[m]]

    def mouse_cb(event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            state.mouse_btns[0] = True

        if event == cv2.EVENT_LBUTTONUP:
            state.mouse_btns[0] = False

        if event == cv2.EVENT_RBUTTONDOWN:
            state.mouse_btns[1] = True

        if event == cv2.EVENT_RBUTTONUP:
            state.mouse_btns[1] = False

        if event == cv2.EVENT_MBUTTONDOWN:
            state.mouse_btns[2] = True

        if event == cv2.EVENT_MBUTTONUP:
            state.mouse_btns[2] = False

        if event == cv2.EVENT_MOUSEMOVE:

            h, w = out.shape[:2]
            dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

            if state.mouse_btns[0]:
                state.yaw += float(dx) / w * 2
                state.pitch -= float(dy) / h * 2

            elif state.mouse_btns[1]:
                dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
                state.translation -= np.dot(state.rotation, dp)

            elif state.mouse_btns[2]:
                dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
                state.translation[2] += dz
                state.distance -= dz

        if event == cv2.EVENT_MOUSEWHEEL:
            dz = math.copysign(0.1, flags)
            state.translation[2] += dz
            state.distance -= dz

        state.prev_mouse = (x, y)


    state = AppState()
    # Configure the pipeline to stream both color and depth
    pipeline = realsense_camera.pipeline
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    try:
        
        # Get stream profile and camera intrinsics
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        colorizer = rs.colorizer()

        cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(state.WIN_NAME, w, h)
        cv2.setMouseCallback(state.WIN_NAME, mouse_cb)

            
        out = np.empty((h, w, 3), dtype=np.uint8)

        while True:
            # Grab camera data
            if not state.paused:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                depth_frame = decimate.process(depth_frame)

                # Grab new intrinsics (may be changed by decimation)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w, h = depth_intrinsics.width, depth_intrinsics.height

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_colormap = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data())

                if state.color:
                    mapped_frame, color_source = color_frame, color_image
                else:
                    mapped_frame, color_source = depth_frame, depth_colormap

                points = pc.calculate(depth_frame)
                pc.map_to(mapped_frame)

                # Pointcloud data to arrays
                v, t = points.get_vertices(), points.get_texture_coordinates()
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
                texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

                # Render
                now = time.time()

                out.fill(0)

                grid(out, (0, 0.5, 1), size=1, n=10)
                frustum(out, depth_intrinsics)
                axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

                if not state.scale or out.shape[:2] == (h, w):
                    pointcloud(out, verts, texcoords, color_source)
                else:
                    tmp = np.zeros((h, w, 3), dtype=np.uint8)
                    pointcloud(tmp, verts, texcoords, color_source)
                    tmp = cv2.resize(
                        tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                    np.putmask(out, tmp > 0, tmp)

                if any(state.mouse_btns):
                    axes(out, view(state.pivot), state.rotation, thickness=4)

                dt = time.time() - now

                cv2.setWindowTitle(
                    state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
                    (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

                # Define the coordinates of the red point (0, 0, 0)
                red_point = np.array(poi) # np.array([0, 0, 0])

                # Render the red point
                if state.scale:
                    proj_red_point = project(view(red_point.reshape(-1, 3)))[0]
                    if not np.isnan(proj_red_point).any():
                        proj_red_point = tuple(proj_red_point.astype(int))
                        cv2.circle(out, proj_red_point, 5, (0, 0, 255), -1)  # Red point

                cv2.imshow(state.WIN_NAME, out)
                key = cv2.waitKey(1)

                if key == ord("r"):
                    state.reset()

                if key == ord("p"):
                    state.paused ^= True

                if key == ord("d"):
                    state.decimate = (state.decimate + 1) % 3
                    decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)

                if key == ord("z"):
                    state.scale ^= True

                if key == ord("c"):
                    state.color ^= True

                if key == ord("s"):
                    cv2.imwrite('./out.png', out)

                if key == ord("e"):
                    points.export_to_ply('./out.ply', mapped_frame)

                if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def get_depth_poi(realsense_camera, x=320, y=240):
    depth_values = []
    for i in range(-2, 2):
        for j in range(-2, 2):
            depth_values.append(get_depth(realsense_camera, x+i, y+j))

    depth_values.sort()

    da = np.array(depth_values)
    da = da[~np.all(da == 0, axis=1)]
    average = np.mean(da, axis=0)
    return average
        

def get_depth(realsense_camera, x=320, y=240):
    # Coordinate change
    x, y = y, x
    dx ,dy, dz = -1, -1, -1

    # Configure depth and color streams
    pipeline = realsense_camera.pipeline
    colorizer = realsense_camera.colorizer
    align = realsense_camera.align

    print(colored("Getting depth", "blue"))
    try:
        # distance = 0
        count = 0
        # file_name = str(time.time()).split('.')[0]

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

            # print(f"!!!1depth from rs: {depth}")
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            # print(f"!!! projection : {dx ,dy, dz}")
            color_image = np.asanyarray(color_frame.get_data())
            color_frame_not_aligned = np.asanyarray(color_frame_not_aligned.get_data())

            depth_colormap = np.asanyarray(
                        colorizer.colorize(depth_frame).get_data())
            img = np.hstack((color_image, depth_colormap))
            # cv2.imshow('Depth Visualization', img)
            # cv2.waitKey(1) 


        return dx ,dy, dz
    
    except Exception as e:
        print(colored(f"Problem in realsense_utils.py->get_depth! {e}", "red"))
        # pipeline.stop()
        return dx ,dy, dz
    
def get_depth_orig(x=320, y=240):
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
            # cv2.imshow('RealSense', color_image)

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
