#!/usr/bin/env python3
import rospy
from pepper_ws.srv import multi_frame
from pipeline import Perception
from realsense_utils import *
from termcolor import colored
from pepper_ws.srv import visual_servo
import rospkg


"""
CMU MRSD Program, course 16-681
Team GetAGrip.AI
Team members: Sridevi Kaza, Jiyoon Park, Shri Ishwaryaa S V, Alec Trela, Solomon Fenton
Rev0: April 3, 2023
Code description: Combined code for the harvest service and the visual servoing service
"""


rospack = rospkg.RosPack()


perception = None
first_request = True

dx, dy, dz = 0, 0, 0
got_depth = False

def handle_multi_frame(req):
    global first_request
    global perception
    # print(colored(f"Request ID: {req.req_id}", "blue"))

    if req.req_id == 0:
        print(colored("Manipulation system requested to start multiframe", "blue"))
        perception.add_frame_to_multi_frame()
        return 1
    else:
        print(colored("Manipulation system requested to process multiframe", "blue"))
        perception.add_frame_to_multi_frame()
        perception.process_multi_frame()
        pepper_found = perception.send_to_manipulator()
        return pepper_found
    
def combined_server():

    global perception
    global dx, got_depth

    rospy.init_node('combined_server')

    rate = rospy.Rate(1) # TODO: Change this to 
    os.chdir(rospack.get_path("pepper_ws"))

    perception = Perception()

    # handle_multi_frame(0)
    # handle_multi_frame(0)
    # handle_multi_frame(0)
    # handle_multi_frame(1)

    s_mf = rospy.Service('/perception/harvest', multi_frame, handle_multi_frame)
    s_vs = rospy.Service('/perception/visual_servo', visual_servo, handle_visual_servoing)

    # handle_visual_servoing(0)

    while not rospy.is_shutdown():
        if got_depth:
            print(colored(f"Visual servo results: x:{dx} y:{dy} z:{dz}", "green"))
            start_time = time.time()
            while time.time() - start_time<1:
                publish_d(dx ,dy, dz)
            got_depth = False
    rospy.spin()


def visual_servoing():

    global got_depth
    
    start_time = time.time()
    img = get_image(perception.rs_camera)
    # print(colored(f"Get img took {time.time()-start_time}", 'cyan'))

    img_name=str(time.time()).split('.')[0]
    img_path = os.getcwd()+'/visual_servoing/'+img_name+'.png'

    cv2.imwrite(img_path, img)

    pepper_of_interest = None
    pepper_of_interest_poi_xyz = (0, 0, 0)

    try:
        print("start initializing yolo")
        start_time = time.time()
        pp = perception.peduncle_detector
        # print("done initializing yolo: ", time.time() - start_time)
        start_time = time.time()
        peduncle_list = pp.predict_peduncle(img_path)
        # print("done predicting yolo: ", time.time() - start_time)
        
        for peduncle_num, peduncle in peduncle_list.items():
            start_time = time.time()
            peduncle.set_point_of_interaction(img.shape, perception.rs_camera)
            # print("done set_poi yolo: ", time.time() - start_time)
            # (dx ,dy, dz) = get_xy_in_realworld(peduncle.poi_px[0], peduncle.poi_px[1]) #TODO
            start_time = time.time()
            (dx, dy, dz) = get_depth(perception.rs_camera, peduncle.poi_px[0], peduncle.poi_px[1])
            print(colored(f"Get depth took {time.time()-start_time}", 'cyan'))
            if pepper_of_interest == None:
                print(colored("first pepper", 'magenta'))
                pepper_of_interest = peduncle
                pepper_of_interest_poi_xyz = (dx ,dy, dz)

            elif int(dx) != 0 and abs(dx)<=abs(pepper_of_interest_poi_xyz[0]):
                print(colored(f"changing pepper because [{dx}] is smaller than {pepper_of_interest_poi_xyz[0]}", 'magenta'))
                pepper_of_interest = peduncle
                pepper_of_interest_poi_xyz = (dx ,dy, dz)
            else:
                print(colored(f"not changing pepper because [{dx}] is bigger than {pepper_of_interest_poi_xyz[0]}", 'magenta'))

        if pepper_of_interest != None:
            # pp.plot_results(peduncle_list, pepper_of_interest.poi_px, pepper_of_interest_poi_xyz)
            got_depth = True

        print(colored(f"pepper of interest: \n, {pepper_of_interest_poi_xyz}", "blue"))
        return pepper_of_interest_poi_xyz

    except Exception as e:
        print(colored(f"Error in detecting pepper: {e}", "red"))
        return pepper_of_interest_poi_xyz
    # get the x, y, z in the realsense axis frame
    # this should be 0, offset of the camera in th rs frame's -z axis 
    # and the z is just not going to work because it dies at 0.15 depth

def publish_d(x, y, z):

    visual_servo_pub = rospy.Publisher('/perception/peduncle/dpoi', Pose, queue_size=10)
    change_pose = Pose()
    # change to the base_link frame 
    change_pose.position.x = float(z) - 0.03 # the end effector is going in too deep
    change_pose.position.y = -float(x)
    change_pose.position.z = -float(y)
    change_pose.orientation.x = 0
    change_pose.orientation.y = 0
    change_pose.orientation.z = 0
    change_pose.orientation.w = 1
    if z > 0.1:
        visual_servo_pub.publish(change_pose)

def handle_visual_servoing(req):

    global dx, dy, dz
    global perception

    print(colored("Starting visual servoing", 'blue'))
    
    if req.req_id == 0:
    # if req == 0:
        (dx ,dy, dz) = visual_servoing(perception.rs_camera)

        (dx ,dy, dz) = (dx-0.03 ,-(dy+0.04), dz) # parameter tuning

        print(colored(f"in realsense world: has to move x, y, z {round(dx, 3), round(dy,3), round(dz,3)}",'blue'))

        if dz<=0.2:
            print(colored("Visual servoing failed :( Returning 0", 'red'))
            return 0
        else:
            print(colored("Visual servoing success :) Returning 1", 'green'))
            return 1



if __name__ == "__main__":
    print(colored("In Service Main", "blue"))
    combined_server()