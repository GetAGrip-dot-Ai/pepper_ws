#!/usr/bin/env python3
import rospy 
from pipeline import Perception
import os
from std_msgs.msg import Float32, Int64MultiArray

if __name__ == '__main__':
    # os.chdir(os.getcwd()+'/pepper_ws/')
    os.chdir('/'.join(__file__.split('/')[:-1]))
    print("current working dir: ",os.getcwd())
    test_img_path = '/realtime'
    pipeline = Perception(test_img_path, 0)
    (x, y) = pipeline.detect_peppers_realtime()
    # rospy.init_node('hi2_node')
    pub = rospy.Publisher('/pp/poi_test', Int64MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        msg = Int64MultiArray()
        msg.data = [int(x*100),int(y*100)]
        pub.publish(msg)


    # pipeline.send_to_manipulator()

'''
input an image
    make a one_frame
        run pepper_fruit detection
        run pepper_peduncle detection
        match pepper
    get peduncle location 

'''
