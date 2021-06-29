#! /usr/bin/env python

import rospy
import sys
import baxter_interface
from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2
from baxter_cv import board_Recognition

from image_geometry.cameramodels import PinholeCameraModel

from baxter_manipulation import IK_move

from baxter_core_msgs.msg import EndpointState


rospy.init_node('Baxter_move')

rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()
left = baxter_interface.Limb('left')
angles = {'left_w0': 0.048546672600166166, 'left_w1': 1.505723257084189, 'left_w2': 0.36137482103163315, 'left_e0': -0.17278111723906756, 'left_e1': 1.055158112709171, 'left_s0': -0.27480749824493955, 'left_s1': -1.0664980586411266}
left.move_to_joint_positions(angles)

gripper = baxter_interface.Gripper('left')
gripper.open()
rospy.sleep(1.0)

arm_move = IK_move('left')

# Camera model
cam_model = PinholeCameraModel()
cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo)
cam_model.fromCameraInfo(cam_info)


# Board recognition
corner = []
while len(corner) < 81:
    current_frame = cv2.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg")
    board_img = board_Recognition(current_frame)
    corner, occupancy_squares, origin_position = board_img.initialize_Board()

print 'origin', origin_position
piece_on = [sq.position for sq in occupancy_squares]
print 'occupancy', piece_on

start = sys.argv[1]
end = sys.argv[2]
print(start, end)
if start not in piece_on:
    print('Wrong starting point. Check again')
else:
    state = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
    current_x = state.pose.position.x
    current_y = state.pose.position.y
    current_z = state.pose.position.z
    qx = state.pose.orientation.x
    qy = state.pose.orientation.y
    qz = state.pose.orientation.z
    qw = state.pose.orientation.w

    us,vs = cam_model.rectifyPoint(origin_position[start])
    xs,ys,zs = cam_model.projectPixelTo3dRay((us,vs))
    print('(x,y,z) = ',(xs,ys,zs))

    ue,ve = cam_model.rectifyPoint(origin_position[end])
    xe,ye,ze = cam_model.projectPixelTo3dRay((ue,ve))
    print('(x,y,z) = ',(xe,ye,ze))

    depth = 0.625556
    real_xs = (xs)*depth
    real_ys = (ys)*depth
    real_xe = (xe)*depth
    real_ye = (ye)*depth


    offset_x = +0.005
    offset_y = +0.005

    # if start[0] == 'E' or start[0] == 'F':
    #     offset_y = 0.020
    #     if start[1] in ['1','2']:
    #         offset_x = 0.005
    # elif start[0] == 'G':
    #     offset_y = -0.010
    #     if start[1] in ['1','2','3']:
    #         offset_x = -0.01
    # elif start[0] == 'H':
    #     offset_y = -0.020
    #     offset_x = -0.010


    joints = arm_move.ik_test(current_x-real_ys +offset_x,current_y-real_xs+offset_y,current_z-0.3,0,qy-0.0099,0,qw+0.005)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.3,0,0.985,0,0.140)
        left.move_to_joint_positions(joints)

    joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.43,0,qy-0.0099,0,qw+0.005)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.43,0,0.980,0,0.140)
        left.move_to_joint_positions(joints)

    # joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.49,qx,qy-0.0099,qz,qw+0.05)
    # if joints is not False:
    #     left.move_to_joint_positions(joints)
    # else: 
    #     joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.49,qx,0.980,qz,0.140)
    #     left.move_to_joint_positions(joints)

    # rospy.sleep(8.0)
    # gripper.close()
    # rospy.sleep(1.0)

    # offset_x = +0.005
    # offset_y = +0.005

    # if end[0] == 'E' or end[0] == 'F':
    #     offset_y = 0.010
    #     if end[1] in ['1','2']:
    #         offset_x = 0.005
    # elif end[0] == 'G':
    #     offset_y = -0.010
    #     if end[1] in ['1','2']:
    #         offset_x = -0.01
    # elif end[0] == 'H':
    #     offset_y = -0.020
    #     offset_x = -0.010

#     joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.3,0,qy-0.0099,0,qw+0.05)
#     if joints is not False:
#         left.move_to_joint_positions(joints)
#     else: 
#         joints = arm_move.ik_test(current_x-real_ys+offset_x,current_y-real_xs+offset_y,current_z-0.3,qx,0.980,qz,0.140)
#         left.move_to_joint_positions(joints)


#     joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.4,qx,qy-0.0099,qz,qw+0.05)
#     if joints is not False:
#         left.move_to_joint_positions(joints)
#     else: 
#         joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.4,qx,0.980,qz,0.140)
#         left.move_to_joint_positions(joints)

#     joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.4,qx,qy-0.0099,qz,qw+0.05)
#     if joints is not False:
#         left.move_to_joint_positions(joints)
#     else: 
#         joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.4,qx,0.980,qz,0.140)
#         left.move_to_joint_positions(joints)

#     joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.487,qx,qy-0.0099,qz,qw+0.05)
#     if joints is not False:
#         left.move_to_joint_positions(joints)
#     else: 
#         joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.487,qx,0.980,qz,0.140)
#         left.move_to_joint_positions(joints)

#     rospy.sleep(8.0)
#     gripper.open()
#     rospy.sleep(1.0)


#     joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.25,qx,qy-0.0099,qz,qw+0.05)
#     if joints is not False:
#         left.move_to_joint_positions(joints)
#     else: 
#         joints = arm_move.ik_test(current_x-real_ye+offset_x,current_y-real_xe+offset_y,current_z-0.25,qx,0.975,qz,0.145)
#         left.move_to_joint_positions(joints)

# left.move_to_joint_positions(angles)





'''
# Transform pixel coordinate to camera coordinate
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

depth = 0.632342
norm = 1.0/z
real_x = (x)*depth
real_y = (y)*depth


arm_move = IK_move('left')

offset_x = +0.010
offset_y = +0.025
square = occupancy_squares[0].position
if square[0] == 'E' or square[0] == 'F' :
    offset_y = 0.010
    if square[1] in ['1','2']:
        offset_x = 0.005
elif square[0] == 'G':
    offset_y = 0.005
    if square[1] in ['1','2']:
        offset_x = -0.005
elif square[0] == 'H':
    offset_y = -0.005
    if square[1] in ['1','2']:
        offset_x = -0.005

joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.25,qx,qx-0.0099,qz,qw+0.01)
if joints is not False:
    left.move_to_joint_positions(joints)
else: 
    joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.25,qx,0.985,qz,0.135)
    left.move_to_joint_positions(joints)

joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.4,qx,qx-0.0099,qz,qw+0.01)
if joints is not False:
    left.move_to_joint_positions(joints)
else: 
    joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.44,qx,0.985,qz,0.135)
    left.move_to_joint_positions(joints)

joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.48,qx,qx-0.0099,qz,qw+0.01)
if joints is not False:
    left.move_to_joint_positions(joints)
else: 
    joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.48,qx,0.985,qz,0.135)
    left.move_to_joint_positions(joints)


rospy.sleep(5)

joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.25,qx,qx-0.0099,qz,qw+0.01)
if joints is not False:
    left.move_to_joint_positions(joints)
else: 
    joints = arm_move.ik_test(current_x-real_y+offset_x,current_y-real_x+offset_y,current_z-0.25,qx,0.985,qz,0.135)
    left.move_to_joint_positions(joints)

left.move_to_joint_positions(angles)
'''
