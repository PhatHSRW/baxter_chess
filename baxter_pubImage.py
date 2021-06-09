#! /usr/bin/env python

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2 
import rospy
import pickle
import baxter_interface
import time
from baxter_interface import CHECK_VERSION

from message_filters import Subscriber, ApproximateTimeSynchronizer

class CameraFeed:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = Subscriber("/cameras/left_hand_camera/image", Image)
        self.image_sub.registerCallback(self.callback)
        
        time.sleep(0.2)

    def callback(self, data):
        self.frame = data

    def get_frame(self):
        current_frame = self.bridge.imgmsg_to_cv2(self.frame)
        cv2.imshow('frame', current_frame)
        cv2.waitKey(0.2)

        return current_frame


rospy.init_node('Baxter_ImgPub')

rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()

left = baxter_interface.Limb('left')

left.move_to_neutral()

# angles = {'left_w0': 0.12757625638048056, 'left_w1': 1.0922045238070204, 'left_w2': 0.1767234144360477, 'left_e0': -0.5911943593181564, 'left_e1': 1.6193382121014699, 'left_s0': 0.002632091639671066, 'left_s1': -1.257569325609626}
angles = {'left_w0': 0.4301782216461116, 'left_w1': 1.3748171781562948, 'left_w2': 0.06659757068732151, 'left_e0': -1.163865659163202, 'left_e1': 1.344250311821102, 'left_s0': 0.3881460130807474, 'left_s1': -1.0802848527443514}
left.move_to_joint_positions(angles)
# Node is subscribing to the video_frames topic

feed = CameraFeed()
frame = feed.get_frame()

rospy.spin()