import roslaunch
import rospy


rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/realsense-ros/realsense2_camera/launch/demo_pointcloud.launch"])
launch.start()
rospy.loginfo("started")



rospy.sleep(10)
# 3 seconds later
launch.shutdown()
print("SHUT DOWN")

