#! /usr/bin/env python

import rospy
import baxter_interface

rospy.init_node('Left_Baxter')
left = baxter_interface.Limb('left')
# angles = left.joint_angles()
# joints = left.joint_names()
# print(angles)
# print(joints)
left.move_to_neutral()

# for joint in joints:
#     angles[joint]=0.0
# print(angles)
# left.move_to_joint_positions(angles)

la = {'left_w0': -0.3183010131225586, 'left_w1': 0.08283496245117188, 'left_w2': -0.1975000262145996, 'left_e0': -2.371150800164795, 'left_e1': 2.125713874383545, 'left_s0': 0.5050631738342285, 'left_s1': -0.5069806498168946}

left.move_to_joint_positions(la)

