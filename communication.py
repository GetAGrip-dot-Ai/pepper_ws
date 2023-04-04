import rospy
from geometry_msgs.msg import Pose

class Communication:
    def __init__(self):
        rospy.init_node('perception', anonymous=True)
        self.poi_pub = rospy.Publisher('/perception/peduncle/poi', Pose, queue_size=10)

    def poi_pub_fn(self, poi, orientation):
        peduncle_pose = Pose()
        peduncle_pose.position.x = poi[0]
        peduncle_pose.position.y = poi[1]
        peduncle_pose.position.z = poi[2]
        # peduncle_pose.orientation = orientation
        rospy.loginfo(peduncle_pose)
        self.poi_pub.publish(peduncle_pose)
