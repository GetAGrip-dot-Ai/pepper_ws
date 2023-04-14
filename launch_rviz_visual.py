import roslaunch
import rospy
import rospkg

rospack = rospkg.RosPack()
pepper_ws_path = rospack.get_path('pepper_ws')
realsense2_camera_path = rospack.get_path('realsense2_camera')

depth_cloud_time_limit = 20

rospy.init_node('launch_rviz_visual', anonymous=True)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [pepper_ws_path+"/launch/demo_pointcloud.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(depth_cloud_time_limit)
launch.shutdown()
print("SHUT DOWN")

