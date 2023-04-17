#!/usr/bin/env python3
import sys
import roslaunch
import rospy
import rospkg
rospack = rospkg.RosPack()

rospy.init_node('marker_rviz', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [os.chdir(rospack.get_path('pepper_ws'))+"/launch/run_marker.launch"])
launch.start()
rospy.loginfo("started")

arg_list = sys.argv
print("arg_list: ", arg_list)

try:
    # launch.spin()
    rospy.sleep(40)

finally:
# After Ctrl+C, stop all nodes from running
    launch.shutdown()
    print("SHUT DOWN")


