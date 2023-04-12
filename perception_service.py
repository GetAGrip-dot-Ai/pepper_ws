#!/usr/bin/env python3

from __future__ import print_function
import os 
import time 
from pepper_ws.srv import harvest
import rospy
from pipeline import Perception

def detect_peppers_realtime():
	os.chdir('/root/catkin_ws/src/pepper_ws/')
	print("current working dir: ",os.getcwd())

	test_img_path = '/realtime'
	pipeline = Perception(test_img_path, 0)
	(x, y) = pipeline.detect_peppers_realtime()
	
	pipeline.send_to_manipulator()

def handle_harvest(req):
	# state = req.req_id
	state = req
	# match req:
	if state == '2':
		print("Detecting peppers")
		detect_peppers_realtime()
		os.system('python3 launch_rviz_visual.py' ) 
		return 1
	else:
		print("Not talking to perception")

def perception_server():
	rospy.init_node('perception_state_server')
	# s = rospy.Service('/perception/harvest', harvest, handle_harvest)
	state = 0
	while state != '0':
		state = input("0:exit\n2:harvest\n")
		handle_harvest(state)
		print("done one")
		
	print("Perception System Ready")
	rospy.spin()

if __name__ == "__main__":
    perception_server()