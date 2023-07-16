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

    def __init__(self):
        if RealsenseCamera._pipeline == None:
            RealsenseCamera._pipeline = rs.pipeline()
        if RealsenseCamera._config == None:
            RealsenseCamera._config = rs.config()
        if RealsenseCamera._colorizer == None:
            RealsenseCamera._colorizer = rs.colorizer()

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

