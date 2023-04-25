#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from communication import Communication
from geometry_msgs.msg import Pose

class MarkerPublisher:
    def __init__(self):
        rospy.init_node('marker_publisher', anonymous=True)
        self.comm = Communication()
        # self.poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)
        self.pepper_tf_sub = rospy.Subscriber('/tf', Pose, self.tf_callback)
        rospy.spin()


    def tf_callback(self,msg):
        try:
            pepper_tf = rospy.get_param("pepper_tf").split(",")
            pepper_tf = (float(pepper_tf[0]), float(pepper_tf[1]), float(pepper_tf[2]))
            # self.poi_rviz_pub.publish(self.comm.make_marker("base_link", point, r=0, b=1, scale=0.5))
            self.comm.single_rviz_marker_poi_realsense_frame(pepper_tf)
        except KeyError:
            print("value not set")


    # marker = comm.make_marker("base_link", r=0, b=1, scale=0.5)
    # marker.points.append(point)

    # while not rospy.is_shutdown():
        # poi_rviz_pub.publish(marker)
MarkerPublisher()



