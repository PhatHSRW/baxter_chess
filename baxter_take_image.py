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



def callback(data):
 
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    cv2.imshow("camera", current_frame)
    
    # # Board recognition
    # board_img = board_Recognition(current_frame)
    # piece_position = board_img.initialize_Board()
    # print 'piece', piece_position
    # for square in board_img.Squares:
    #     if square.state == True:
    #         print(square.position, square.roi)

    # # Transform pixel coordinate to camera coordinate
    # cam_model = PinholeCameraModel()
    # cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo)
    # cam_model.fromCameraInfo(cam_info)
    # u,v = cam_model.rectifyPoint(piece_position)
    # x,y,z = cam_model.projectPixelTo3dRay((u,v))
    # print('(x,y,z) = ',(x,y,z)) 

    # state = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
    # current_x = state.pose.position.x
    # current_y = state.pose.position.y
    # current_z = state.pose.position.z
    # qx = state.pose.orientation.x
    # qy = state.pose.orientation.y
    # qz = state.pose.orientation.z
    # qw = state.pose.orientation.w

    # depth = 0.646
    # norm = 1.0/z
    # real_x = (x)*depth
    # real_y = (y)*depth


    # arm_move = IK_move('left')
    # joints = arm_move.ik_test(current_x-real_y+0.118,current_y-real_x,current_z-0.3,qx,qy,qz,qw)
    # left.move_to_joint_positions(joints)
    # joints = arm_move.ik_test(current_x-real_y+0.118,current_y-real_x,current_z-0.4,0,1,0,0)
    # left.move_to_joint_positions(joints)
    # # rospy.sleep(20)


    # gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)s
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg", current_frame)


rospy.init_node('Baxter_Image')

rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()

left = baxter_interface.Limb('left')

left.move_to_neutral()


# ___________________960x600____________________
'''Working angles'''
angles = {'left_w0': 0.16863027943720657, 'left_w1': 1.2706910845487016, 'left_w2': 0.2717977449830381, 'left_e0': -0.5868248231392752, 'left_e1': 1.24579654840489, 'left_s0': 0.027081806815071312, 'left_s1': -1.12648712794576}
angles = {'left_w0': -0.0327752462622192, 'left_w1': 1.2590981605393181, 'left_w2': 0.40196026865212825, 'left_e0': -0.19000319513230696, 'left_e1': 1.2675203401693111, 'left_s0': -0.22153771044883655, 'left_s1': -1.1877354842689298}

left.move_to_joint_positions(angles)
# Node is subscribing to the video_frames topic
rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()


