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
        rospy.init_node('pp_p_communication_node', anonymous=True)
        self.poi_pub = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/perception/pepper/bbox', Obstacle, queue_size=10)
        self.poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)
        self.listener = tf.TransformListener()
        
    def poi_pub_fn(self, poi, orientation):

        now = rospy.Time.now()
        self.listener.waitForTransform("/rs_ee", "/base_link", now, rospy.Duration(10.0))
        # now = now -9
        (trans,rot) = self.listener.lookupTransform("/base_link", "/rs_ee", now)
        print("?????????",trans)
        print("^^", poi)
        print("!!!", rot)

        r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
        # import pdb; pdb.set_trace()

        H = np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1)))
        row = np.array([0,0,0,1])
        H = np.vstack((H,row))
        print("H: ", H)
        poi = list(poi)
        poi.append(1)
        print("POI: ", poi)
        point = np.array(H) @ np.array(poi).T
        peduncle_pose = Pose()
        # # import pdb; pdb.set_trace()
        # pp = np.array(r.as_matrix()) @ np.array(list(poi)).T+np.array(trans).T
        # peduncle_pose.position.x = -trans[0] +poi[0]
        # peduncle_pose.position.y = trans[1]-poi[1]
        # peduncle_pose.position.z = trans[2] +poi[2]
        # peduncle_pose.orientation = orientation
        # import pdb; pdb.set_trace()
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
        obstacle_msg = Obstacle()
        for obstacle in obstacles:
            pose = Pose()
            pose.position.x = obstacle.xywh[0]/100
            pose.position.y = obstacle.xywh[1]/100
            pose.position.z = 0.2

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [obstacle.xywh[2]/100, obstacle.xywh[3]/100, 0.1]
            obstacle_msg.pose.append(pose)
            obstacle_msg.primitive.append(primitive)
            
        rospy.loginfo(obstacle_msg)
        self.obstacle_pub.publish(obstacle_msg)

    def poi_rviz_pub_fn(self, peppers):
        marker = Marker()
        marker.type = 8
        marker.header.frame_id = "rs_ee"
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            
            point = Point()
            point.x = poi[0]
            point.y = poi[1]
            point.z = poi[2] # convert to meter
            marker.points.append(point)
        
        # print(marker.points)

        # rospy.loginfo(marker)
        self.poi_rviz_pub.publish(marker)

    def poi_rviz_pub_fn_base_link(self, peppers):
        marker = Marker()
        marker.type = 8
        marker.header.frame_id = "base_link"
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            now = rospy.Time.now()
            self.listener.waitForTransform("/rs_ee", "/base_link", now, rospy.Duration(10.0))
            # now = now -9
            (trans,rot) = self.listener.lookupTransform("/base_link", "/rs_ee", now)

            r = R.from_quat([rot[0], rot[1], rot[2], rot[3]]) # rotation part of R
            # import pdb; pdb.set_trace()
            H = np.hstack((r.as_matrix(),np.array(trans).reshape(3, 1)))
            row = np.array([0,0,0,1])
            H = np.vstack((H,row))
            print("H: ", H)
            poi = list(poi)
            poi.append(1)
            print("POI: ", poi)
            point = np.array(H) @ np.array(poi).T
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2] # convert to meter
            marker.points.append(p)

        
        # print(marker.points)

        # rospy.loginfo(marker)
        self.poi_rviz_pub.publish(marker)

