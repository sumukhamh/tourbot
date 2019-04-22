#!/usr/bin/env python
'''
 * Name: utils.py
 
 * Author:  Shashank Shastry 	scshastr@eng.ucsd.edu
 	        Sumukha Harish 	ssumukha@eng.ucsd.edu
 	        Sai Adithya		schittur@eng.ucsd.edu
 
 * Date: 03/18/2019
 
 * Description: This is the helper module for the main.py 
                script. It contains the necessary functions
                for laoding data, processing it and outputting
                it from the turtlebot
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


#########################################################################################################
# GLOBAL VARIABLES
#########################################################################################################

# Chhange these variables to have different functionalities
funny_flag=True
interact_flag=False

path='/home/turtlebot/alpha_ws_new/src/tourbot/tourbot/files/'
battery_level = 0.0
current_pose=0
sound_client=SoundClient()
rospy.sleep(1)

# Define quaternions for four directions
quats={'right':Quaternion(0,0,0,1),
    'bottom':Quaternion(0,0,-0.707,0.707),
    'left':Quaternion(0,0,1,0),
    'top':Quaternion(0,0,0.707,0.707)}

# Define checkpoints on the map
check_points=[

    {'title':'emergency_exit','point':(2.08,-4.23),'ori':'top','formal':7,'funny':9},

    {'title':'sitting_area','point':(5.34,-0.98),'ori':'right','formal':19,'funny':19},

    {'title':'conference_room','point':(10.1,-7.81),'ori':'bottom','formal':11,'funny':17},

    {'title':'graduate_office','point':(11.7,-15.3),'ori':'bottom','formal':7,'funny':7},

    {'title':'nvsl_lab','point':(16.1,-33.9),'ori':'bottom','formal':7,'funny':8},

    {'title':'ubi_lab','point':(8.85,-33.6),'ori':'top','formal':19,'funny':17},

    {'title':'embedded_lab','point':(5.18,-15.9),'ori':'top','formal':24,'funny':46}

    ]

# Variables for face detection
face_time_prev=0
cascade_cfg_path = '/home/turtlebot/alpha_files/turtlebot_human_detection_interaction/alpha_pkg/src/haarcascade_frontalface_alt.xml'
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')
face_cascade.load(cascade_cfg_path)
bridge = CvBridge()

#########################################################################################################
# FUNCTIONS
#########################################################################################################

# Play video
def open_file(filename,time):
    opener = "xdg-open"
    p=subprocess.Popen([opener,filename])
    rospy.sleep(time)
    p=subprocess.Popen([opener,path+'walking_final.mp4'])

# Play sound 
def speak(str):
    sound_client.say(str)
    rospy.sleep(len(str)*0.1)

# Parse content from narration.txt file and load to data
def load_interaction(path):
    file=open(path)
    data={}
    title=''
    for line in file:
        temp=line.split('*')
        if(len(temp)>1):
            title=temp[1].split()[0]
        temp=line.split('#')
        if(len(temp)>1):
            data[title]={'text':temp[1]}
        temp=line.split('@')
        if(len(temp)>1):
            data[title]['funny']=temp[1]
    return data

# Interact at checkpoints by displaying audio and video 
def DisplayAndSound(data):
    '''
    # Use this in case of playing audio and displaying images only
    plt.figure(figsize=(20,20))
    plt.imshow(data['img'])
    plt.show(block=False)
    speak(data['text'])
    if(funny_flag):
        speak(data['funny'])
    plt.close('all')
    '''

    # Use this in case of playing video
    if(funny_flag):
        open_file(path+'videos_funny/'+data['title']+'.mp4',data['funny'])
    else:
        open_file(path+'videos_formal/'+data['title']+'.mp4',data['formal'])

# Command line interface at checkpoints
def InteractWindow(str1):
    speak(str1+', press 1, otherwise, press 0.')
    c = input(str1+' press 1\nOtherwise, press 0\n')
    # os.system('clear')
    return (c == 1)

# Interactions
def Interact(data):
    interact_set={'sitting_area','graduate_office','ubi_lab'}
    if(interact_flag and data['title'] in interact_set):
        if(data['title']=='sitting_area'):
            DisplayAndSound(data)
            choice=InteractWindow('If you want me to wait while you check out the massage chair')
            if(choice):
                speak('I will wait for 5 seconds')
                rospy.sleep(5)

        if(data['title']=='graduate_office'):
            choice=InteractWindow('if you want to hear about the graduate offices')
            if(choice):
                DisplayAndSound(data)

        if(data['title']=='ubi_lab'):
            DisplayAndSound(data)
            choice=InteractWindow('If you want me to wait while you check out the lobby')
            if(choice):
                choice=InteractWindow('If you want to get some snacks at Chez bob')
                if(choice):
                    speak('Sorry, I cannot wait that long')
                else:
                    speak('Alright, I will wait for 5 seconds')
                    rospy.sleep(5)
    else:
        DisplayAndSound(data)

