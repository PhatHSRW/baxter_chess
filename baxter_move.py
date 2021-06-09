#! /usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2 
import numpy as np
from baxter_cv import board_Recognition
import math

from image_geometry.cameramodels import PinholeCameraModel

from baxter_manipulation import IK_move

from baxter_core_msgs.msg import EndpointState

# Board recognition
board_img = board_Recognition(current_frame)
piece_position = board_img.initialize_Board()
print 'piece', piece_position
for square in board_img.Squares:
    if square.state == True:
        print(square.position, square.roi)

# Transform pixel coordinate to camera coordinate
cam_model = PinholeCameraModel()
cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo)
cam_model.fromCameraInfo(cam_info)
u,v = cam_model.rectifyPoint(piece_position)
x,y,z = cam_model.projectPixelTo3dRay((u,v))
print('(x,y,z) = ',(x,y,z)) 

state = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
current_x = state.pose.position.x
current_y = state.pose.position.y
current_z = state.pose.position.z
qx = state.pose.orientation.x
qy = state.pose.orientation.y
qz = state.pose.orientation.z
qw = state.pose.orientation.w

depth = 0.646
norm = 1.0/z
real_x = (x)*depth
real_y = (y)*depth


arm_move = IK_move('left')
joints = arm_move.ik_test(current_x-real_y+0.118,current_y-real_x,current_z-0.3,qx,qy,qz,qw)
left.move_to_joint_positions(joints)
joints = arm_move.ik_test(current_x-real_y+0.118,current_y-real_x,current_z-0.4,0,1,0,0)
left.move_to_joint_positions(joints)
# rospy.sleep(20)