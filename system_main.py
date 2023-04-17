#!/usr/bin/env python3

from pepper_ws.srv import harvest
import rospy
from pipeline import Perception
from pepper_ws.srv import multi_frame
import rospkg
import roslaunch
state = -100
launch = None

def launch_all():
	global launch
	rospack = rospkg.RosPack()
	pepper_ws_path = rospack.get_path('pepper_ws')

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch = roslaunch.parent.ROSLaunchParent(uuid, [pepper_ws_path+"/launch/launch_all.launch"])
	# launch = roslaunch.parent.ROSLaunchParent(uuid, [pepper_ws_path+"/launch/run_perception_realtime.launch"])
	launch.start()

def handle_main_service(req):
	global launch
	# if req.req_id == 0:
	if req == '1':
		print("STARTING EVEYTHING")
		launch_all()
	elif req == '2':
		print("SHUTTING DOWN EVERYTHING")
		launch.shutdown()
		print("EVERYTHING SHUTDOWN-ED")
		# launch = None
	else:
		print("DOES ABSOLUTELY NOTHING PLEASE PUT SOMETHING VALID")


def perception_server():
	rospy.init_node('perception_state_server')
	state = 0
	# s = rospy.Service('/perception/multi_frame', multi_frame, handle_main_service)
	while state != '0':
		state = input("0:exit\n1:start\n2:end\n")
		handle_main_service(state)
		print("done one")
		
	print("Perception System Ready")
	rospy.spin()

def system_state_callback(msg):
	global state
	state = msg.data

if __name__ == "__main__":
    perception_server()