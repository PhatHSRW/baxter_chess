#! /usr/bin/env python

from sensor_msgs.msg import Range

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

from baxter_core_msgs.msg import EndpointState

import tf, math, time


class IK_Arm:
    def __init__(self, limb, verbose=True):
        self.limb_name = limb
        self.verbose = verbose
        self.limb = baxter_interface.Limb(limb)

    def current_pose(self):
        state = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
        self.current_x = state.pose.position.x
        self.current_y = state.pose.position.y
        self.current_z = state.pose.position.z
        self.current_qx = state.pose.orientation.x
        self.current_qy = state.pose.orientation.y
        self.current_qz = state.pose.orientation.z
        self.current_qw = state.pose.orientation.w

        return self.current_x,self.current_y,self.current_z, self.current_qx, self.current_qy, self.current_qz, self.current_qw


    def ik_request(self, x,y,z, qx,qy,qz,qw):
        ns = "ExternalTools/" + self.limb_name + "/PositionKinematicsNode/IKService"
        self.iksrv_client = rospy.ServiceProxy(ns, SolvePositionIK)
        iksrv_request = SolvePositionIKRequest()

        header = Header(stamp=rospy.Time.now(), frame_id='base')

        pos = Pose()
        pos.position.x = x
        pos.position.y = y
        pos.position.z = z
        pos.orientation.x = qx
        pos.orientation.y = qy
        pos.orientation.z = qz
        pos.orientation.w = qw


        iksrv_request.pose_stamp.append(PoseStamped(header=header, pose=pos))
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = self.iksrv_client(iksrv_request)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        iksrv_request.SEED_USER: 'User Provided Seed',
                        iksrv_request.SEED_CURRENT: 'Current Joint Angles',
                        iksrv_request.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format((seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
            # print "Response Message:\n", resp
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints


rospy.init_node("IK_Baxter")

rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
rs.enable()

left = baxter_interface.Limb('left')
left.move_to_neutral()

left_manipulation = IK_Arm("left")

angles = left_manipulation.ik_request(0.7073,0.5333,0.25,0,1,0,0)
left.move_to_joint_positions(angles)

x,y,z, qx,qy,qz,qw = left_manipulation.current_pose()
print(x,y,z,qx,qy,qz,qw)


sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
depth = sensor.range
# print('depth is ' + str(depth))

while True:
    joints = left_manipulation.ik_request(x,y,z-0.05, 0,1,0,0)
    left.move_to_joint_positions(joints)
    x,y,z, qx,qy,qz,qw = left_manipulation.current_pose()
    
    sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
    depth = sensor.range
    if depth < 0.3:
        print("Stop")
        time.sleep(3)
        sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
        d_min = sensor.range
        print 'd_min ', d_min
        x2,y2,z2, qx2,qy2,qz2,qw2 = left_manipulation.current_pose()
        print 'z2 = ', z2
        print(qx2,qy2,qz2,qw2)
        break

# angles = {'left_w0': 0.048546672600166166, 'left_w1': 1.505723257084189, 'left_w2': 0.36137482103163315, 'left_e0': -0.17278111723906756, 'left_e1': 1.055158112709171, 'left_s0': -0.27480749824493955, 'left_s1': -1.0664980586411266}
angles = {'left_w0': 0.07439806812744142, 'left_w1': 1.7038691581970216, 'left_w2': 0.2803349886657715, 'left_e0': -0.15224759302368165, 'left_e1': 0.5809952227478028, 'left_s0': -0.3984515091979981, 'left_s1': -0.7861651528930664}
left.move_to_joint_positions(angles)
x1,y1,z1, qx1,qy1,qz1,qw1 = left_manipulation.current_pose()
print 'z1 = ', z1
print(qx1,qy1,qz1,qw1)
print 'deltaZ = ', z1-z2


e_angle = tf.transformations.euler_from_quaternion([qx1+qx2,qy1+qy2,qz1+qz2,qw1+qw2], axes='rxyz')
print(e_angle)
d2 = (z1-z2 + d_min)/math.cos(e_angle[1])
print('depth ', d2)

# rospy.spin()


# z_board = -0.187        z_coordinate of board respect to baxter
