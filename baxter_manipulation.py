#! /usr/bin/env python

import rospy
import struct

import baxter_interface
from baxter_interface import CHECK_VERSION

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from std_msgs.msg import Header

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest, SolvePositionIKResponse

class IK_move:

    def __init__(self, limb, verbose=True):
        self.limb_name = limb
        self.verbose = verbose
        self._limb = baxter_interface.Limb(limb)
        ns = '/ExternalTools/' + limb + '/PositionKinematicsNode/IKService'
        self.iksrv_client = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

        # self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        # self._init_state = self._rs.state().enabled
        # self._rs.enable()

    def ik_test(self,x,y,z, qx,qy,qz,qw):
        iksrv_request = SolvePositionIKRequest()
        header = Header(stamp=rospy.Time.now(), frame_id = 'base')

        position = Pose()
        position.position.x = x
        position.position.y = y
        position.position.z = z
        position.orientation.x = qx
        position.orientation.y = qy
        position.orientation.z = qz
        position.orientation.w = qw

        iksrv_request.pose_stamp.append(PoseStamped(header=header, pose=position))
        try:
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
            if self.verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints



        