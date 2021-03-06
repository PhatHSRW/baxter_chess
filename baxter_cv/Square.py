import cv2
import numpy as np
import math

class SquareClass:

	def __init__(self, image, c1, c2, c3, c4, position, state=False):

		# Corners
		self.c1 = c1
		self.c2 = c2
		self.c3 = c3
		self.c4 = c4

		# Position
		self.position = position

		# npArray of corners
		self.contour = np.array([[c1,c2,c4,c3]],dtype=np.int32)

		# Center of square
		M = cv2.moments(self.contour, True)
		cx = int(M['m10'] / M['m00'])
		cy = int(M['m01'] / M['m00'])

		# ROI for image differencing
		self.roi = (cx, cy)
		self.radius = 2
		self.emptyColor = self.roiColor(image)
		self.state = state

	def draw(self, image, color,thickness=2):
		square_contour = np.array(self.contour).reshape((-1,1,2)).astype(np.int32)
		cv2.drawContours(image, [square_contour], 0, color, 3)

	def drawROI(self, image, color, thickness = 1):

		cv2.circle(image, self.roi, self.radius, color,thickness)


	def roiColor(self, image):
		# Initialise mask
		mask = np.zeros((image.shape[0], image.shape[1]), np.uint8)
		# Draw the ROI circle on the mask
		cv2.circle(mask, self.roi, self.radius, (255, 255, 255), -1)
		# Find the average color
		average_raw = cv2.mean(image, mask=mask)[::-1]
		# Need int format so reassign variable
		average = (int(average_raw[1]), int(average_raw[2]), int(average_raw[3]))

		return average

	def classify(self, image):

		rgb = self.roiColor(image)

		sum = 0
		for i in range(0,3):
			sum += (self.emptyColor[i] - rgb[i])**2


		cv2.putText(image, self.position, (self.roi[0]-5,self.roi[1]+15),cv2.FONT_HERSHEY_COMPLEX,0.6,(0,255,255),1)

