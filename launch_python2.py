# #!/usr/bin/env python3
# import roslaunch

# package = 'realsesnse2_camera'
# executable = 'demo_pointcloud.launch'
# node = roslaunch.core.Node(package, executable)

# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()

# process = launch.launch(node)
# print (process.is_alive())
# process.stop()

import roslaunch
import rospy

rospy.init_node('en_Mapping2', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/pepper_ws/launch/run_perception_realtime.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(10)
launch.shutdown()
