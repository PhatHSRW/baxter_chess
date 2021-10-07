import math
import cv2
import numpy as np
import imutils
from Line import LineClass
from Square import SquareClass

check =  True

class board_Recognition:
	'''
	This class handles the initialization of the board by finding chess board, lines and squares, etc.
	Work perfect for resolution 960x600
	rosrun baxter_tools camera_control.py -o left_hand_camera -r 960x600
	'''

	def __init__(self, image):

		self.image = image

	def initialize_Board(self):

		# Threshold the image to binary
		adaptiveThresh,img = self.thresh_Image(self.image)

		# Separate chess board from raw image
		mask, points = self.initialize_mask(adaptiveThresh,img)

		# Perspective transform the chess board
		perspective_output, trans_matrix = self.perspective(mask, points)

		# Find edges
		edges, colorEdges = self.findEdges(perspective_output)

		# Find lines
		horizontal, vertical = self.findLines(edges,colorEdges.copy())

		# Find corners
		self.corners = self.findCorners(horizontal, vertical, colorEdges)
		print(len(self.corners))

		# Find squares
		squares = self.findSquares(self.corners, colorEdges)

		# Find Occupancy squares
		occupancy_squares = self.occupancySquares(perspective_output, edges, squares)

		# Find all square in pixel uv coordinate
		origin_positions = self.square_on_image(mask, squares, trans_matrix)

		return self.corners, occupancy_squares, origin_positions


	def thresh_Image(self,image):
		'''
		Resizes and converts the image to black and white for simpler analysis
		'''
		# resize image
		img = imutils.resize(image)
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# pixels above the threshold value are white and those below are black
		adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 55, 2)

		if not check:
			cv2.imshow("Adaptive Thresholding", adaptiveThresh)
			cv2.waitKey(1)

		return adaptiveThresh, img

	def initialize_mask(self, adaptiveThresh,img):
		'''
		Detect border of chessboard and put all unnecessary pixel (environment outside chessboard) into zero, 
		meaning black color.
		'''

		# Find contours which are closed polygons
		if cv2.__version__[0]=="3":
			_, contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		else: contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# Create copy of original image
		# imgContours = img.copy()

		for c in range(len(contours)):
			# Area
			area = cv2.contourArea(contours[c])
			# Perimenter
			perimeter = cv2.arcLength(contours[c], True)
			#the largest square is always the largest ratio -> chess board
			if c ==0:
				Lratio = 0
			if perimeter > 0:
				ratio = area / perimeter
				if ratio > Lratio:
					largest=contours[c]
					Lratio = ratio
					Lperimeter=perimeter
					Larea = area
			else:
					pass

		# Draw contours
		cv2.drawContours(self.image, [largest], -1, (0,0,255), 2)
		if check:
			cv2.imshow("Chess Board",self.image)
			cv2.waitKey(0)

		# Epsilon parameter needed to fit contour to polygon
		epsilon = 0.1 * Lperimeter
		# Approximates a polygon from chessboard edge
		chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)
		chessboardEdge = np.squeeze(chessboardEdge)
		print "4 corners: ", chessboardEdge

		# Create new all black image
		mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')
		# Copy the chessboard edges as a filled white polygon size of chessboard edge
		cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
		# Assign all pixels that are white (i.e the polygon, i.e. the chessboard)
		extracted = np.zeros_like(img)
		extracted[mask == 255] = img[mask == 255]

		if not check:
			cv2.imshow("mask",extracted)
			cv2.waitKey(1)
			
		return extracted, chessboardEdge

	def perspective(self,image, points):

		pts1 = np.float32(points)
		if points[0][0]**2 + points[0][1]**2 > points[1][0]**2 + points[1][1]**2:
			pts2 = np.float32([[500,0],[0,0],[0,500],[500,500]])
		else: pts2 = np.float32([[0,0],[0,500],[500,500],[500,0]])

		Matrix = cv2.getPerspectiveTransform(pts1,pts2)

		# print(Matrix)
		transform = cv2.warpPerspective(image,Matrix,(500,500))
		output = transform[20:transform.shape[1]-20, 20:transform.shape[0]-20]
		histogram = cv2.equalizeHist(cv2.cvtColor(output,cv2.COLOR_BGR2GRAY))

		if not check:
			cv2.imshow("transform",output)
			cv2.imshow("histogram",histogram)
			cv2.waitKey(0)
			
		return output, Matrix


	def findEdges(self, image):
		'''
		Finds edges in the image in order to find lines
		'''
		# Find edges
		image = cv2.boxFilter(image,0,(5,5), normalize = True)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# image = cv2.medianBlur(image,5)
		# image = cv2.GaussianBlur(image,(7,7),0)
		# image = cv2.bilateralFilter(image,5,50,50)
		edges = cv2.Canny(image, 50, 100, None,3)

		if check:
			cv2.imshow("Canny", edges)
			cv2.waitKey(0)
			
		# Convert edges image to BGR
		colorEdges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

		return edges,colorEdges

	def findLines (self, edges, output):
		'''
		Finds the lines in the image and classify into vertical or horizontal list
		'''
		# Infer lines based on edges (HoughLinesP)
		lines = cv2.HoughLinesP(edges, 1,  np.pi/180, 50, np.array([]), 70, 100)		# 50, 50, 60
		# print(lines.shape)

		horizontal = []
		vertical = []

		# Draw lines
		if cv2.__version__[0]=="2":
			for line in lines[0]:
				cv2.line(output, (line[0], line[1]), (line[2], line[3]), (0,255,0),2)
				[x1,y1,x2,y2] = line
				newLine = LineClass(x1,y1,x2,y2)
				if newLine.orientation == 'horizontal':
					horizontal.append(newLine)
				else:
					vertical.append(newLine)
		else: 
			for line in lines:
				cv2.line(output, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0,255,0),2)
				[[x1,y1,x2,y2]] = line
				newLine = LineClass(x1,y1,x2,y2)
				if newLine.orientation == 'horizontal':
					horizontal.append(newLine)
				else:
					vertical.append(newLine)

		if not check:
			cv2.imshow("Lines",output)
			cv2.waitKey(1)
			
		return horizontal, vertical

	def findCorners (self, horizontal, vertical, colorEdges):
		'''
		Finds corners by searching intersections of horizontal and vertical lines.
		'''
		# Find corners (intersections of lines)
		corners = []
		for v in vertical:
			for h in horizontal:
				s1,s2 = v.find_intersection(h)
				corners.append([s1,s2])

		# remove duplicate corners
		finalCorners = []
		for c in corners:
			matchingFlag = False
			for d in finalCorners:
				if math.sqrt((d[0]-c[0])*(d[0]-c[0]) + (d[1]-c[1])*(d[1]-c[1])) < 45:
					matchingFlag = True
					break
			if not matchingFlag:
				finalCorners.append(c)

		for d in finalCorners:
			cv2.circle(colorEdges, (d[0],d[1]), 4, (0,0,255),2)

		if not check:
			cv2.imshow("Corners",colorEdges)
			cv2.waitKey()

		return finalCorners

	def findSquares(self, corners, colorEdges):
		'''
		Finds the squares of the chessboard 
		'''

		# sort corners by row
		corners.sort(key=lambda x: x[0])
		rows = [[],[],[],[],[],[],[],[],[]]
		r = 0
		for c in range(0, 81):
			if c > 0 and c % 9 == 0:
				r = r + 1

			rows[r].append(corners[c])

		letters = ['A','B','C','D','E','F','G','H']
		numbers = ['1','2','3','4','5','6','7','8']
		self.Squares = []
		
		# sort corners by column
		for r in rows:
			r.sort(key=lambda y: y[1])
		
		# initialize squares
		for r in range(0,8):
			for c in range (0,8):
				c1 = rows[r][c]
				c2 = rows[r][c + 1]
				c3 = rows[r + 1][c]
				c4 = rows[r + 1][c + 1]

				position = letters[r] + numbers[7-c]
				newSquare = SquareClass(colorEdges,c1,c2,c3,c4,position)
				newSquare.draw(colorEdges,(0,0,255),1)
				newSquare.drawROI(colorEdges,(0,0,255),2)
				newSquare.classify(colorEdges)
				self.Squares.append(newSquare)

		if not check:
			#Show image with squares and ROI drawn and position labelled
			cv2.imshow("Squares", colorEdges)
			cv2.waitKey(1)

		return self.Squares

	def occupancySquares(self, image, edge, Squares):
		'''Checking which squares having pieces on it'''
		# Before binarization, it is necessary to correct the nonuniform illumination of the background.
		image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		se=cv2.getStructuringElement(cv2.MORPH_RECT , (3,3))
		bg=cv2.morphologyEx(image, cv2.MORPH_DILATE, se, iterations=4)
		out_gray=cv2.divide(image, bg, scale=255)
		out_binary=cv2.threshold(out_gray, 0, 255, cv2.THRESH_OTSU )[1]

		kernel5 = np.ones((5,5),np.uint8)
		kernel3 = np.ones((3,3),np.uint8)
		erosion = cv2.erode(out_binary,kernel5, iterations = 4)

		# square = Squares[0]
		# roi = square.roi
		# print(roi)
		# print(erosion[roi[1]+109,roi[0]+375])

		occupancy_Squares = []

		for square in Squares:
			black = 0
			white = 0
			if square.position in ['A1', 'A2', 'A3']:
				for dx in range(-20,10):
					for dy in range(-10,20):
						if erosion[square.roi[1]+dy, square.roi[0]+dx] == 0:
							black +=1
						else: white +=1
				if black > 0.4*900:
					square.state = True
					occupancy_Squares.append(square)
			else:
				for dx in range(-17,18):
					for dy in range(-12,18):
						if erosion[square.roi[1]+dy, square.roi[0]+dx] == 0:
							black +=1
						else: white +=1
				if black > 0.6*1050:
					square.state = True
					occupancy_Squares.append(square)

		print 'pieces on those squares: ', [sq.position for sq in occupancy_Squares]

		if not check:
			cv2.imshow('erosion', erosion)
			cv2.imshow('nonuniform illumination', out_gray)
			# cv2.imshow('erosion2', erosion2)
			cv2.waitKey(0)

		return occupancy_Squares

	def square_on_image(self, image, squares, trans_matrix):

		inv_maxtrix = np.linalg.inv(trans_matrix)
		origin_points = {}
		for square in squares:
			perspective_point = np.array([[[square.roi[0],square.roi[1]]]],dtype=np.float32)
			origin_point = cv2.perspectiveTransform(perspective_point, inv_maxtrix)
			origin_point = origin_point.flatten()
			origin_point[0] = origin_point[0]+20
			origin_point[1] = origin_point[1]+20
			origin_points[square.position] = origin_point
			cv2.circle(self.image, (origin_point[0], origin_point[1]), 2, (0,0,255), 2)

		if not check:
			cv2.circle(self.image,((480, 288)),2,(0,255,0),2)
			cv2.imshow('Piece Detection', self.image)
			cv2.waitKey(0)

		return origin_points
		