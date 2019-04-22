#!/usr/bin/env python
'''
 * Name: main.py
 
 * Author:      Shashank Shastry 	scshastr@eng.ucsd.edu
 	            Sumukha Harish 	ssumukha@eng.ucsd.edu
 	            Sai Adithya		schittur@eng.ucsd.edu
 
 * Date:        03/18/2019
 
 * Description: This is the main script for running the
                tourbot node that is responsible for 
                making the turtlebot move around as a 
                tour guide. 
  
                Topics subscribed to:
                1. "/usb_cam/image_raw" 
                2. "/mobile_base/sensors/core"
                3. "/amcl_pose"
                
                Topics published:
                1. "/initialpose"
    
* Usage:        roscore
                roslaunch tourbot tourbot_node.launch
                python main.py

* References:   https://github.com/markwsilliman/turtlebot
                https://pythonprogramming.net/haar-cascade-object-detection-python-opencv-tutorial/
'''

import rospy
import cv2
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from kobuki_msgs.msg import SensorState
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os, sys, subprocess, signal

from utils import funny_flag, path, check_points, quats, bridge, face_cascade
import utils


# Class for moving to goal
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, data, pub):
        # Send a goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        self.goal_sent = True
        
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = Pose(Point(data['point'][0], data['point'][1], 0.000),quats[data['ori']])

        # Start moving
        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(100)) 

        state = self.move_base.get_state()

        if success and state == GoalStatus.SUCCEEDED:
            utils.Interact(data)
            self.goal_sent = False
            initial_pos_pub(pub,{'pose':(data['point'][0],data['point'][1]),'ori':quats[data['ori']]})

            if((rospy.Time.now()-face_time_prev)>rospy.Duration.from_sec(90)):
                print('Face not detected, tour ended')
                self.move_base.cancel_goal()
                return False

        else:
            utils.speak('Please clear the way! If I am stuck, please give me a nudge')
            initial_pos_pub(pub,{'pose':(current_pose.position.x,current_pose.position.y),
                'ori':current_pose.orientation})
            return self.goto(data,pub)

        return True

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

# Publish pose to the robot
def initial_pos_pub(pub,pose_ori):
    start_pos = PoseWithCovarianceStamped()
    # Filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    # Filling payload with relevant information gathered from subscribing
    # to /initialpose topic published by RVIZ via rostopic echo /initialpose
    start_pos.pose.pose.position.x = pose_ori['pose'][0]
    start_pos.pose.pose.position.y = pose_ori['pose'][1]
    start_pos.pose.pose.position.z = 0.0
    start_pos.pose.pose.orientation = pose_ori['ori']
    start_pos.pose.covariance=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.04]

    rospy.sleep(2)
    pub.publish(start_pos)

# Callback for battery level
def SensorPowerEventCallback(battery_msg):
    kobuki_base_max_charge = 200
    global battery_level
    battery_level = float(battery_msg.battery)*100/float(kobuki_base_max_charge)

# Callback from USB cam
def CamCallback(image_msg):
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # Detect faces from the input image
    faces = face_cascade.detectMultiScale(gray, 1.3, 2, minSize=(10,10), maxSize=(300,300))

    # Reset face_time_prev to current time
    global face_time_prev
    if len(faces) > 0:
        face_time_prev = rospy.Time.now()

# Callback of current pose
def PoseCallback(pose_msg):
    # Update current_pose
    global current_pose
    current_pose = pose_msg.pose.pose

# Define parameters in the beginning
def set_parameters():
    rospy.set_param('/usb_cam/pixel_format', 'yuyv')
    rospy.set_param('/move_base/planner_frequency',2)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_x',0.2)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_y',0.2)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_theta',0.02)
    rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.18)
    rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',0.04)
    rospy.set_param('/move_base/DWAPlannerROS/max_vel_theta',0.02)
    rospy.set_param('/move_base/DWAPlannerROS/min_vel_theta',-0.02)
    rospy.set_param('/move_base/DWAPlannerROS/min_in_place_vel_theta',0.02)
    rospy.set_param('/move_base/global_costmap/inflation_layer/inflation_radius',0.1)
    rospy.set_param('/move_base/local_costmap/inflation_layer/inflation_radius',0.1)
    rospy.set_param('/move_base/global_costmap/inflation_layer/cost_scaling_factor',4)
    rospy.set_param('/move_base/local_costmap/inflation_layer/cost_scaling_factor',4)
    rospy.set_param('/move_base/local_costmap/robot_radius',0.5)
    rospy.set_param('/move_base/DWAPlannerROS/occdist_scale',2.2) # 0.5 default
    rospy.set_param('/move_base/controller_frequency',7) # 5 default
    rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.20) # 0.15 default
    rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 0.5) # 0.3 default

#########################################################################################################
# MAIN
#########################################################################################################

if __name__ == '__main__':
    try:
        # Initialize tourbot_node
        rospy.init_node('tourbot_node', anonymous=False)
        face_time_prev = rospy.Time.now()

        # Initialize publishers and subscribers
        pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        cam_sub = rospy.Subscriber('/usb_cam/image_raw',Image,CamCallback)
        power_sub = rospy.Subscriber("/mobile_base/sensors/core",SensorState,SensorPowerEventCallback)
        pose_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,PoseCallback)

        # Initial localization
        initial_pose_ori = {'pose':(3.25,-8.47),'ori':quats['top']}       
        initial_pos_pub(pub,initial_pose_ori)

        # Set parameters before start of the mission
        set_parameters()

        # Instantiate
        navigator = GoToPose()

        # Start of tour message
        '''
        # Use this if using audio and images for communication
        sounds = utils.load_interaction(path+'narration.txt')
        for data in check_points:
            if(sounds[data['title']]):
                data['text']=sounds[data['title']]['text']
                data['funny']=sounds[data['title']]['funny']
            img=plt.imread(path+'images/'+data['title']+'.jpg')
            data['img']=img
        utils.speak('Welcome to the tour! It would be my pleasure to show you around! Please follow me')
        '''
        if(funny_flag):
            utils.open_file(path+'videos_funny/start.mp4',12)
        else:
            utils.open_file(path+'videos_formal/start.mp4',8)

        # Go to each checkpoint
        for check_point in check_points:
            success = navigator.goto(check_point,pub)
            if(not success):
                rospy.loginfo("Tour ended, The base failed to reach the desired pose")
                break

        # End of tour message
        rospy.loginfo("Tour completed!")
        utils.speak('Tour complete, thank you! battery level is {} percentage'.format(battery_level))

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

