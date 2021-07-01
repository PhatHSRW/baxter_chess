#! /usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2 
from baxter_cv import board_Recognition
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

    cv2.imwrite("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg", current_frame)
    cv2.circle(current_frame,(477,286),10,(0,0,255),2)
    cv2.circle(current_frame,(480,288),10,(0,255,0),2)

    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)

rospy.init_node('Baxter_Image')

# Node is subscribing to the video_frames topic
rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()


