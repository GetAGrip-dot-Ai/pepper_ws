#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf
rospy.init_node('hi', anonymous=True)
poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)

try:
    poi = rospy.get_param("poi").split(",")
    print("GOT POI", poi) 
    poi = [float(poi[0]), float(poi[1]), float(poi[2])]

    listener = tf.TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform("/rs_ee", "/base_link", now, rospy.Duration(10.0))
    (trans,rot) = listener.lookupTransform("/base_link", "/rs_ee", now)

    r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
    H = np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1)))
    row = np.array([0,0,0,1])
    H = np.vstack((H,row))
    poi = list(poi)
    poi.append(1)
    point = np.array(H) @ np.array(poi).T

except KeyError:
    print("value not set")
print("Hi at least we are here")

marker = Marker()
marker.type = 8
marker.header.frame_id = "base_link"
marker.color.a = 1.0
marker.color.b = 1.0
marker.scale.x = 0.07
marker.scale.y = 0.07

point_marker = Point()
point_marker.x = point[0]
point_marker.y = point[1]
point_marker.z = point[2] # convert to meter
marker.points.append(point_marker)

while not rospy.is_shutdown():
    # rospy.loginfo(marker)
    poi_rviz_pub.publish(marker)




