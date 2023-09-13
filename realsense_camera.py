import rospy
import pyrealsense2 as rs
from termcolor import colored


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Class for a realsense camera and its properties
"""



class RealsenseCamera:
    _pipeline = None
    _config = None
    _colorizer = None
    _align = None
    _is_running = False  # Flag to track pipeline status

    def __init__(self):
        # Check if the pipeline is already running
        if RealsenseCamera._is_running:
            print(colored("Realsense pipeline is already running.", "yellow"))
            return

        if RealsenseCamera._pipeline is None:
            RealsenseCamera._pipeline = rs.pipeline()
        if RealsenseCamera._config is None:
            RealsenseCamera._config = rs.config()
        if RealsenseCamera._colorizer is None:
            RealsenseCamera._colorizer = rs.colorizer()
        if RealsenseCamera._align is None:
            RealsenseCamera._align = rs.align(rs.stream.color)

        pipeline_wrapper = rs.pipeline_wrapper(RealsenseCamera._pipeline)
        pipeline_profile = RealsenseCamera._config.resolve(pipeline_wrapper)
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

        RealsenseCamera._config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            RealsenseCamera._config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            RealsenseCamera._config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        RealsenseCamera._pipeline.start(RealsenseCamera._config)

        align_to = rs.stream.color
        self._align = rs.align(align_to)

        rospy.sleep(3)

        # Update the flag to indicate that the pipeline is running
        RealsenseCamera._is_running = True

        print(colored("Realsense initialization done!", "green"))


    @property
    def pipeline(self):
        return RealsenseCamera._pipeline


    @property
    def config(self):
        return RealsenseCamera._config

    @property
    def colorizer(self):
        return RealsenseCamera._colorizer
    
    @property
    def align(self):
        return self._align

