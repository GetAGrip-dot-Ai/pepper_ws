#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from pepper_ws.msg import Obstacle
import tf
from scipy.spatial.transform import Rotation as R
import numpy as np
from termcolor import colored


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Communicates with other subsystems. It publishes the POI and the obstacles. 
"""


class Communication:
    def __init__(self):
        # rospy.init_node('pp_p_communication_node', anonymous=True)
        self.poi_pub = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/perception/pepper/bbox', Obstacle, queue_size=10)
        self.poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)
        self.listener = tf.TransformListener()
        self.transform_from_base = [[0, 0, 0], [0, 0, 0, 1]]


    def publish_poi(self, poi, orientation):

        peduncle_pose = Pose()
        peduncle_pose.position.x = float(poi[0])
        peduncle_pose.position.y = float(poi[1])
        peduncle_pose.position.z = float(poi[2])
        peduncle_pose.orientation.x = 0
        peduncle_pose.orientation.y = 0
        peduncle_pose.orientation.z = 0
        peduncle_pose.orientation.w = 1
        # rospy.loginfo(peduncle_pose)
        self.poi_pub.publish(peduncle_pose)


    def obstacle_pub_fn(self, obstacles):
        obstacle_msg = Obstacle()

        for obstacle in obstacles:
            point = obstacle.xyz
            pose = Pose()
            pose.position.x = float(point.x)
            pose.position.y = float(point.y)
            pose.position.z = float(point.z)

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.12, 0.12, 0.12]
            obstacle_msg.pose.append(pose)
            obstacle_msg.primitive.append(primitive)
            
        rospy.loginfo(obstacle_msg)
        self.obstacle_pub.publish(obstacle_msg)


    def rviz_marker_poi_realsense_frame(self, peppers):
        marker = self.make_marker("realsense_frame", r=1, g=0, b=1, scale=0.1)

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            point = Point()
            point.x = poi[0]
            point.y = poi[1]
            point.z = poi[2] # convert to meter
            marker.points.append(point)
        
        self.poi_rviz_pub.publish(marker)
   

    def rviz_marker(self, point, r=1, g=0, b=0):
        marker = self.make_marker("base_link", r=r, g=g, b=b, scale=0.1)
        marker.points.append(point)
        self.poi_rviz_pub.publish(marker)


    def rviz_marker_rs(self, point, r=1, g=0, b=1):
        marker = self.make_marker("realsense_frame", r=r, g=g, b=b, scale=0.1)
        marker.points.append(point)
        self.poi_rviz_pub.publish(marker)


    def single_rviz_marker_poi_realsense_frame(self, poi):
        marker = self.make_marker("base_link", r=0, g=1, b=0, scale=0.04)

        point = Point()
        point.x = poi[0]
        point.y = poi[1]
        point.z = poi[2] 
        marker.points.append(point)
        
        self.poi_rviz_pub.publish(marker)


    def rviz_marker_poi_base_link(self, peppers):
        marker = self.make_marker("base_link", r=1, g=0, b=0)

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            p = self.transform_to_base_link(poi)
            marker.points.append(p)

        self.poi_rviz_pub.publish(marker)


    def transform_to_base_link(self, point_in_relative_frame, trans, rot):
        rate = rospy.Rate(10)

        r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
        H = np.vstack((np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1))),np.array([0,0,0,1])))
        point_in_relative_frame = list(point_in_relative_frame) + [1]
        point = np.array(H) @ np.array(point_in_relative_frame).T

        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2] # convert to meter
        print(colored(f"poi relative to the base_link:\n{p}", "light_cyan"))
        return p


    def make_marker(self, frame_id = "base_link", r=1, g=0, b=0, scale=0.03):
        marker = Marker()
        marker.type = 8
        marker.header.frame_id = frame_id
        marker.color.a = 0.5
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.scale.x = scale
        marker.scale.y = scale
        return marker