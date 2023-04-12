import rospy
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from pepper_ws.msg import Obstacle
import tf
from scipy.spatial.transform import Rotation as R
import numpy as np

class Communication:
    def __init__(self):
        # rospy.init_node('pp_p_communication_node', anonymous=True)
        self.poi_pub = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/perception/pepper/bbox', Obstacle, queue_size=10)
        self.poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)
        self.listener = tf.TransformListener()
        
    def publish_poi(self, poi, orientation):

        point = self.transform_to_base_link(poi)

        peduncle_pose = Pose()
        peduncle_pose.position.x = float(point[0])
        peduncle_pose.position.y = float(point[1])
        peduncle_pose.position.z = float(point[2])
        peduncle_pose.orientation.x = 0
        peduncle_pose.orientation.y = 0
        peduncle_pose.orientation.z = 0
        peduncle_pose.orientation.w = 1
        rospy.loginfo(peduncle_pose)
        self.poi_pub.publish(peduncle_pose)

    def obstacle_pub_fn(self, obstacles):
        now = rospy.Time.now()
        self.listener.waitForTransform("/realsense_frame", "/base_link", now, rospy.Duration(10.0))
        (trans, rot) = self.listener.lookupTransform("/base_link", "/realsense_frame", now)

        r = R.from_quat([rot[0], rot[1], rot[2], rot[3]])

        H = np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1)))
        row = np.array([0, 0, 0, 1])
        H = np.vstack((H, row))

        obstacle_msg = Obstacle()
        for obstacle in obstacles:
            point = np.array(H) @ np.array([obstacle.xyz[0], obstacle.xyz[1], obstacle.xyz[2], 1]).T
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.12, 0.12, 0.12]
            obstacle_msg.pose.append(pose)
            obstacle_msg.primitive.append(primitive)
            
        rospy.loginfo(obstacle_msg)
        self.obstacle_pub.publish(obstacle_msg)

    def rviz_marker_poi_realsense_frame(self, peppers):
        marker = make_marker("realsense_frame", r=0, g=1, b=0, scale=0.04)

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            point = Point()
            point.x = poi[0]
            point.y = poi[1]
            point.z = poi[2] # convert to meter
            marker.points.append(point)
        
        self.poi_rviz_pub.publish(marker)

        if not rospy.has_param('poi') and poi:
            poi_str = f"{poi[0]},{poi[1]},{poi[2]}"
            rospy.set_param('poi', poi_str)
            print("set poi:", poi_str)

    def rviz_marker_poi_base_link(self, peppers):
        marker = make_marker("base_link", r=1, g=0, b=0)

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            p = self.transform_to_base_link(poi)
            marker.points.append(p)

        # rospy.loginfo(marker)
        self.poi_rviz_pub.publish(marker)

    def transform_to_base_link(self, point_in_relative_frame):
        now = rospy.Time.now()
        self.listener.waitForTransform("/realsense_frame", "/base_link", now, rospy.Duration(10.0))
        # now = now -9
        (trans,rot) = self.listener.lookupTransform("/base_link", "/realsense_frame", now)

        r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
        H = np.vstack((np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1))),np.array([0,0,0,1])))
        point_in_relative_frame = list(point_in_relative_frame) + [1]
        point = np.array(H) @ np.array(point_in_relative_frame).T

        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2] # convert to meter
        return p
def make_marker(frame_id = "base_link", r=1, g=0, b=0, scale=0.03):
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