#! /usr/bin/env python

import rospy
import baxter_interface
import sys

rospy.init_node('Reading_angles')
if sys.argv[1] == "l" or sys.argv[1] == "left":
	limb = baxter_interface.Limb('left')
else: limb = baxter_interface.Limb('right')
anglesR = limb.joint_angles()
print(anglesR)
