#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Int64MultiArray
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import matplotlib.pyplot as plt

from pipeline import Perception

if not hasattr(rs2, 'intrinsics'):
    import pyrealsense2.pyrealsense2 as rs2


class PerceptionNode:
    def __init__(self, image_topic, aligned_topic, depth_pub_topic):
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(image_topic, msg_Image, self.cameraCallback)
        self.aligned_sub = rospy.Subscriber(aligned_topic, msg_Image, self.alignedTopicCallback)
        self.depth_img = None
        self.rgb_img = None
        self.depth_pub = rospy.Publisher(depth_pub_topic, Int64MultiArray, queue_size=10)

    def alignedTopicCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.depth_img = cv_image
            depth_msg = Int64MultiArray()
            depth_msg.data = cv_image.flatten().tolist()
            self.depth_pub.publish(depth_msg)

        except CvBridgeError as e:
            print(e)
            return    

    def cameraCallback(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
            
        except CvBridgeError as e:
            print(e)
            return


def main():
    image_topic = '/camera/color/image_raw'
    aligned_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_pub_topic = '/camera/pp/depth'

    test_img_path = '/test'

    os.chdir('/'.join(__file__.split('/')[:-1]))
    pn = PerceptionNode(image_topic, aligned_topic,depth_pub_topic)
    pipeline = Perception(test_img_path, 0)
    
    pipeline.detect_peppers_in_folder()
    pipeline.send_to_manipulator()
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()