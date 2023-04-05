import rospy
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from pepper_ws.msg import Obstacle

class Communication:
    def __init__(self):
        rospy.init_node('pp_p_communication_node', anonymous=True)
        self.poi_pub = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/perception/pepper/bbox', Obstacle, queue_size=10)
        self.poi_rviz_pub = rospy.Publisher('/perception/peduncle/poi_rviz', Marker, queue_size=10)
        
    def poi_pub_fn(self, poi, orientation):
        peduncle_pose = Pose()
        peduncle_pose.position.x = poi[0]
        peduncle_pose.position.y = poi[1]
        peduncle_pose.position.z = poi[2]
        # peduncle_pose.orientation = orientation
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

        for pepper in peppers:
            poi = pepper.pepper_peduncle.poi
            
            point = Point()
            point.x = poi[0]
            point.y = poi[1]
            point.z = poi[2]
            marker.points.append(point)

        # rospy.loginfo(marker)
        self.poi_rviz_pub.publish(marker)

