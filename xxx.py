#! /usr/bin/env python

# from baxter_cv.mask_board import initialize_Board
# import cv2
import cv2


# print(current_frame)
current_frame  = cv2.imread("./frame.jpg")
cv2.imshow("camera", current_frame)
cv2.waitKey(1)


# e_angle = tf.transformations.euler_from_quaternion([0,0.975,0,0.09], axes='rzxy')
#print(e_angle)

