#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import pickle
import cv2 
import numpy as np


class ImageConverter:
    
    def __init__(self, ros_node):
        
        self.bridge = CvBridge()
        self.topic = "/cameras/left_hand_camera/image"
        self.ros_node = str(ros_node)
 
        rospy.init_node(self.ros_node)
        print("DEBUG")

    def callback(self, data):
        # Output debugging information to the terminal
        rospy.loginfo("receiving video frame")

        # Convert ROS Image message to OpenCV image
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(data)
            with open("current_frame.p", "wb") as file:
                pickle.dump(self.current_frame, file)
            
        except CvBridgeError as e:
            rospy.loginfo(e)
        except Exception as e:
            print(e)

        cv2.imshow("camera", self.current_frame)
        cv2.waitKey(1)

    def subscribe(self):
        self.image_sub = rospy.Subscriber(self.topic, Image, self.callback)


image = ImageConverter("test")
image.subscribe()
rospy.spin()

    
        



    

