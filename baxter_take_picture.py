#! /usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np
# from baxter_cv.mask_board import initialize_Board
import math

def callback(data):
 
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    # initialize_Board(current_frame)
    cv2.imshow("camera", current_frame)
    # gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/{}.jpg".format(data.header.stamp),
        current_frame)
    



rospy.init_node('Video_Sub_BaxterLeft')

rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()

left = baxter_interface.Limb('left')

left.move_to_neutral()

# -----------------640x400-----------------
# angles = {'left_w0': -0.10622816943969728, 'left_w1': -1.262849682183838, 'left_w2': 0.06481068821411133, 'left_e0': -0.0613592314453125, 'left_e1': 2.3186119582397464, 'left_s0': -0.8643981729858399, 'left_s1': -0.6676651371643066}
# angles = {'left_w0': -0.00728640873413086, 'left_w1': 0.9859661502868653, 'left_w2': -0.033364082098388675, 'left_e0': 0.08590292402343751, 'left_e1': 0.9955535302001953, 'left_s0': -0.8973787598876953, 'left_s1': -0.930742841986084}
# angles = {'left_w0': -0.06902913537597656, 'left_w1': -0.3613689746948242, 'left_w2': -0.028378644543457034, 'left_e0': 0.11965050131835939, 'left_e1': 1.9407041416198732, 'left_s0': -0.979995275177002, 'left_s1': -0.9280583756103516}
# -----------------640x400-----------------

# -----------------960x600-----------------
# angles = {'left_w0': -0.002300971179199219, 'left_w1': 1.1474176280273438, 'left_w2': -0.011888351092529297, 'left_e0': 0.07976700087890626, 'left_e1': 0.2442864401916504, 'left_s0': -0.8651651633789063, 'left_s1': -0.35473305679321293}
# angles = {'left_w0': -0.0015339807861328126, 'left_w1': 0.9579710009399415, 'left_w2': 0.0015339807861328126, 'left_e0': -0.0015339807861328126, 'left_e1': 0.7466651476501466, 'left_s0': -0.8233641869567871, 'left_s1': -0.6979612576904297}
# angles = {'left_w0': -0.00843689432373047, 'left_w1': -0.26269420962524415, 'left_w2': 0.0015339807861328126, 'left_e0': -0.0019174759826660157, 'left_e1': 1.5523885555664063, 'left_s0': -0.8233641869567871, 'left_s1': -0.6204952279907227}
# -----------------960x600-----------------

# ________1280x800________
# angles = {'left_w0': 0.02914563493652344, 'left_w1': 1.0691846079345704, 'left_w2': -0.04141748122558594, 'left_e0': 0.03413107249145508, 'left_e1': 0.9223059476623536, 'left_s0': -0.8390874900146484, 'left_s1': -0.9928690638244629}
# angles = {'left_w0': 0.027228158953857422, 'left_w1': 0.06596117380371094, 'left_w2': -0.03911651004638672, 'left_e0': 0.031446606115722656, 'left_e1': 1.664752648150635, 'left_s0': -0.8191457397949219, 'left_s1': -0.9821311983215333}
angles = {'left_w0': 0.026844663757324222, 'left_w1': 1.6785584752258302, 'left_w2': -0.03834951965332031, 'left_e0': 0.031446606115722656, 'left_e1': 0.48627190920410157, 'left_s0': -0.826815643725586, 'left_s1': -0.9134855581420899}

left.move_to_joint_positions(angles)

# Node is subscribing to the video_frames topic
rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()


