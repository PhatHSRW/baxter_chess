#! /usr/bin/env python

from sensor_msgs.msg import Range

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
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

# angles = {'left_w0': 0.026844663757324222, 'left_w1': 1.6785584752258302, 'left_w2': -0.03834951965332031, 'left_e0': 0.031446606115722656, 'left_e1': 0.48627190920410157, 'left_s0': -0.826815643725586, 'left_s1': -0.9134855581420899}
angles = left_manipulation.ik_request(0.78,0.25,0.25,0,0.98,0,0)
left.move_to_joint_positions(angles)

x,y,z, qx,qy,qz,qw = left_manipulation.current_pose()
print(x,y,z,qx,qy,qz,qw)


sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
depth = sensor.range
print('depth is ' + str(depth))

while depth > 0.30:
    x,y,z, qx,qy,qz,qw = left_manipulation.current_pose()
    joints = left_manipulation.ik_request(x,y,z-0.01, qx,qy,qz,qw)
    left.move_to_joint_positions(joints)

    sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
    depth = sensor.range
    print(depth)
    if depth < 0.305:
        print("Stop")
        break

x,y,z, qx,qy,qz,qw = left_manipulation.current_pose()
joints = left_manipulation.ik_request(x,y,z-0.01, qx,0.989,qz,0.145)
left.move_to_joint_positions(joints)
sensor = rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
depth = sensor.range
print(depth)

# rospy.spin()