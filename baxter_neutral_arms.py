#! /usr/bin/env python

import rospy
import baxter_interface

rospy.init_node('Neutral_Baxter')

rs = baxter_interface.RobotEnable()
rs.enable()

left = baxter_interface.Limb('left')
right = baxter_interface.Limb('right')
left.move_to_neutral()
# right.move_to_neutral()

