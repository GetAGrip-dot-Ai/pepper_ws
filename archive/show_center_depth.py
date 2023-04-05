#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
import sys
import os
import numpy as np
import pyrealsense2 as rs2
import matplotlib.pyplot as plt

if not hasattr(rs2, 'intrinsics'):
    import pyrealsense2.pyrealsense2 as rs2


class ImageListener:
    def __init__(self, image_topic, aligned_topic, depth_pub_topic) :
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(image_topic, msg_Image, self.cameraCallback)
        self.aligned_sub = rospy.Subscriber(aligned_topic, msg_Image, self.alignedTopicCallback)
        self.depth_img = None
        self.rgb_img = None
        self.depth_pub = rospy.Publisher(depth_pub_topic, Float32)

    def alignedTopicCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.depth_img = cv_image
            # h, w = self.depth_img.shape # 480 640
            h, w = 480, 640
            # import ipdb; ipdb.settrace();
            plt.imshow(cv_image)
            # plt.xlim()

            for wi in range(0, w-10, 60):
                for hi in range(0, h-1, 20):
                    # print('limits', plt.xlim(), '====', plt.ylim())
# limits (-0.5, 639.5) ==== (479.5, -0.5)

                    print(wi, hi)
                    plt.text(wi, hi, self.depth_img[hi, wi])
            # plt.plot(100, 200, 'ro')
            plt.savefig('hi.png')
            plt.cla()
            plt.clf()

        except CvBridgeError as e:
            print(e)
            return    

    def cameraCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.rgb_img = cv_image
        except CvBridgeError as e:
            print(e)
            return


def main():
    image_topic = '/camera/color/image_raw'
    aligned_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_pub_topic = '/camera/pp/depth'
    
    
    listener = ImageListener(image_topic, aligned_topic,depth_pub_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()