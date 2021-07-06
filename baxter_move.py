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
from pixel_to_Baxter3D import CameraModel


def quaternion(square):
    qx, qy, qz, qw = 0,0.98,0,0.14
    if square in ['G1', 'H1', 'F1']:
        qx = 0.0191420169913
        qy = 0.998437294909
        qz = 0.0110917468564
        qw = 0.0513178767221
    elif square in ['A8']:
        qx = -0.0734912710183
        qy = 0.966114259207
        qz = 0.0519552658506
        qw = 0.241915112366
    return qx, qy, qz, qw

def offset(square):
    offset_x = -0.005
    offset_y = 0
    if square in ['A8']:
        offset_x = -0.0075
    if square[1] == '1':
        offset_x = 0.0075
    return offset_x, offset_y

rospy.init_node('Baxter_move')

rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()
left = baxter_interface.Limb('left')
angles = {'left_w0': 0.07439806812744142, 'left_w1': 1.7038691581970216, 'left_w2': 0.2803349886657715, 'left_e0': -0.15224759302368165, 'left_e1': 0.5809952227478028, 'left_s0': -0.3984515091979981, 'left_s1': -0.7861651528930664}
left.move_to_joint_positions(angles)

gripper = baxter_interface.Gripper('left')
gripper.open()
rospy.sleep(1.0)

arm_move = IK_move('left')

# Camera model
cam_model = PinholeCameraModel()
cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo)
cam_model.fromCameraInfo(cam_info)

# cam_model = CameraModel()


# Board recognition
corner = []
while len(corner) < 81:
    current_frame = cv2.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg")
    board_img = board_Recognition(current_frame)
    corner, occupancy_squares, origin_position = board_img.initialize_Board()   

# print 'origin', origin_position
piece_on = [sq.position for sq in occupancy_squares]

start = sys.argv[1]
end = sys.argv[2]
print(start, end)

if start not in piece_on:
    print('Wrong starting point. Check again')
else:
    state = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
    start_x = state.pose.position.x
    start_y = state.pose.position.y
    start_z = state.pose.position.z

    us,vs = cam_model.rectifyPoint(origin_position[start])
    xs,ys,zs = cam_model.projectPixelTo3dRay((us,vs))
    print('(x,y,z) = ',(xs,ys,zs))

    ue,ve = cam_model.rectifyPoint(origin_position[end])
    xe,ye,ze = cam_model.projectPixelTo3dRay((ue,ve))
    print('(x,y,z) = ',(xe,ye,ze))

    # z_chessboard = -0.188, z_cam = 0.403 => h = 0.591
    # anpha =  0.860914 0.0893687 0.0862841
    depth = 0.591     # 0.60855

    real_xs = (xs/zs)*depth
    real_ys = (ys/zs)*depth
    real_xe = (xe/ze)*depth
    real_ye = (ye/ze)*depth

    '--------------------Pick step--------------------'
    # offset and adjust
    offset_x, offset_y = offset(start)
    qx,qy,qz,qw = quaternion(start)

    # X Y Z is baxter coordinate
    X = start_x-real_ys+offset_x
    Y = start_y-real_xs+offset_y
    Z = start_z

    joints = arm_move.ik_test(X, Y, Z-0.3, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.3, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    joints = arm_move.ik_test(X, Y, Z-0.405, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.41, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    joints = arm_move.ik_test(X,Y,Z-0.49,0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X,Y,Z-0.492, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    rospy.sleep(2.0)
    gripper.close()
    rospy.sleep(1.0)
  

    joints = arm_move.ik_test(X, Y, Z-0.3,0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.3, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)
    
    if start in ['H1', 'G1', 'F1']:
        joints = arm_move.ik_test(X+0.05, Y+0.05, Z-0.3, 0,0.98,0,0.17)
        left.move_to_joint_positions(joints)


    '--------------------Place step--------------------'

    # offset and adjust
    offset_x, offset_y = offset(end)
    qx,qy,qz,qw = quaternion(end)

    # X Y Z is baxter coordinate
    X = start_x-real_ye+offset_x
    Y = start_y-real_xe+offset_y
    Z = start_z

    joints = arm_move.ik_test(X, Y, Z-0.3, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.3, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    joints = arm_move.ik_test(X, Y, Z-0.405, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.41, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    joints = arm_move.ik_test(X, Y, Z-0.485, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X-0.005, Y, Z-0.49, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    rospy.sleep(2.0)
    gripper.open()
    rospy.sleep(1.0)


    joints = arm_move.ik_test(X, Y, Z-0.4, 0,0.98,0,0.17)
    if joints is not False:
        left.move_to_joint_positions(joints)
    else: 
        joints = arm_move.ik_test(X, Y, Z-0.4, qx, qy, qz, qw)
        left.move_to_joint_positions(joints)

    if end in ['H1', 'G1', 'F1']:
        joints = arm_move.ik_test(X+0.05, Y+0.05, Z-0.28, 0,0.98,0,0.17)
        left.move_to_joint_positions(joints)

left.move_to_joint_positions(angles)


    # end_state()
    # circle = (0,0)
    # while True:
    #     image = cv2.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg")
    #     image = image[100:400,300:700]
    #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #     img_blur = cv2.medianBlur(gray, 3)
    #     # image = io.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg")
    #     circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1, img_blur.shape[0]/5, param1=80, param2=12, minRadius=20, maxRadius=30)
    #     r_max = 1

    #     if circles is not None:
    #         for i in circles[0, :]:
    #             if i[2] > r_max:
    #                 r_max = i[2]
    #                 circle = (i[0], i[1])
                    
    #     calib_x = 0
    #     calib_y = 0            
    #     if abs(circle[0]-180) > 20:
    #         if circle[0] < 180:
    #             calib_y = +0.01
    #         elif circle[0] > 180: 
    #             calib_y = -0.01

    #     if abs(circle[1]-160) > 20:
    #         if circle[1] < 160:
    #             calib_x = +0.01
    #         elif circle[1] > 160: 
    #             calib_x = -0.01
                
    #     joints = arm_move.ik_test(current_x+calib_x,current_y+calib_y,current_z,qx,qy,qz,qw)
    #     left.move_to_joint_positions(joints)
    #     end_state()
                

    #     if abs(circle[0]-180) <= 20 and abs(circle[1]-160) <= 20:
    #         break