import math
import cv2
import numpy as np
import imutils
from Line import LineClass
from Square import SquareClass

debug =  False


class board_Recognition:
	'''rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
	This class handles the initialization of the board. It analyzes
	the empty board finding its border, lines, corners, squares...
	'''

	def __init__(self, image):

		self.image = image

	def initialize_Board(self):

		# Binarize the photo
		adaptiveThresh,img = self.clean_Image(self.image)

		# Black out all pixels outside the border of the chessboard
		mask, points = self.initialize_mask(adaptiveThresh,img)

		perspective_output, trans_matrix = self.perspective(mask, points)

		# Find edges
		edges, colorEdges = self.findEdges(perspective_output)

		# Find lines
		horizontal, vertical = self.findLines(edges,colorEdges)

		# Find corners
		self.corners = self.findCorners(horizontal, vertical, colorEdges)
		print(len(self.corners))
		# print(corners)

		# Find squares
		squares = self.findSquares(self.corners, colorEdges)

		# Find Occupancy squares
		occupancy_squares = self.occupancySquares(perspective_output,squares)

		self.findPiece(mask, occupancy_squares, trans_matrix)

		return squares


	def clean_Image(self,image):
		'''
		Resizes and converts the photo to black and white for simpler analysis
		'''
		# resize image
		img = imutils.resize(image)
		
		# Convert to grayscale
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# Setting all pixels above the threshold value to white and those below to black
		# Adaptive thresholding is used to combat differences of illumination in the picture
		adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 55, 2)
		if debug:
			# Show thresholded image
			cv2.imshow("Adaptive Thresholding", adaptiveThresh)
			cv2.waitKey(0)

		return adaptiveThresh, img

	def initialize_mask(self, adaptiveThresh,img):
		'''
		Finds border of chessboard and blacks out all unneeded pixels
		'''

		# Find contours (closed polygons)
		if cv2.__version__[0]=="3":
			_, contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		else: contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# Create copy of original image
		imgContours = img.copy()

		for c in range(len(contours)):
			# Area
			area = cv2.contourArea(contours[c])
			# Perimenter
			perimeter = cv2.arcLength(contours[c], True)
				# Filtering the chessboard edge / Error handling as some contours are so small so as to give zero division
				#For test values are 70-40, for Board values are 80 - 75 - will need to recalibrate if change
				#the largest square is always the largest ratio
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
		cv2.drawContours(imgContours, [largest], -1, (0,0,255), 2)
		if not debug:
			# Show image with contours drawn
			cv2.imshow("Chess Board",imgContours)
			cv2.waitKey(0)

		# Epsilon parameter needed to fit contour to polygon
		epsilon = 0.1 * Lperimeter
		# Approximates a polygon from chessboard edge
		chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)
		chessboardEdge = np.squeeze(chessboardEdge)
		print(chessboardEdge)

		# Create new all black image
		mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')
		# Copy the chessboard edges as a filled white polygon size of chessboard edge
		cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
		# Assign all pixels that are white (i.e the polygon, i.e. the chessboard)
		extracted = np.zeros_like(img)
		extracted[mask == 255] = img[mask == 255]

		if not debug:
			# Show image with mask drawn
			cv2.imshow("mask",extracted)
			cv2.waitKey(0)
			
		return extracted, chessboardEdge

	def perspective(self,image, points):

		pts1 = np.float32(points)
		if points[0][0]**2 + points[0][1]**2 > points[1][0]**2 + points[1][1]**2:
			pts2 = np.float32([[500,0],[0,0],[0,500],[500,500]])
		else: pts2 = np.float32([[0,0],[0,500],[500,500],[500,0]])

		Matrix = cv2.getPerspectiveTransform(pts1,pts2)

		# print(Matrix)
		transform = cv2.warpPerspective(image,Matrix,(500,500))
		output = transform[20:transform.shape[0]-20, 20:transform.shape[1]-20]
		# histogram = cv2.equalizeHist(cv2.cvtColor(output,cv2.COLOR_BGR2GRAY))

		if not debug:
			# Show image with mask drawn
			cv2.imshow("transform",output)
			# cv2.imshow("histogram",histogram)
			cv2.waitKey(0)
			
		return output, Matrix


	def findEdges(self, image):
		'''
		Finds edges in the image. Edges later used to find lines and so on
		'''
	
		# Find edges
		image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		image = cv2.boxFilter(image,0,(5,5), normalize = True)
		# image = cv2.medianBlur(image,5)
		# image = cv2.GaussianBlur(image,(7,7),0)
		# image = cv2.bilateralFilter(image,5,50,50)
		edges = cv2.Canny(image, 40, 80, None,3)
		if debug:
			#Show image with edges drawn
			cv2.imshow("Canny", edges)
			cv2.waitKey(0)
			

		# Convert edges image to grayscale
		colorEdges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

		return edges,colorEdges

	def findLines (self, edges, output):
		'''
		Finds the lines in the photo and sorts into vertical and horizontal
		'''
		
		# Infer lines based on edges (HoughLinesP)
		lines = cv2.HoughLinesP(edges, 1,  np.pi / 180, 50, np.array([]), 50, 60)		# 50, 50, 60
		print(lines.shape)

		horizontal = []
		vertical = []

		# Draw lines
		if cv2.__version__[0]=="2":
			for line in lines[0]:
				# cv2.line(output, (line[0], line[1]), (line[2], line[3]), (0,255,0),2)
				[x1,y1,x2,y2] = line
				newLine = LineClass(x1,y1,x2,y2)
				if newLine.orientation == 'horizontal':
					horizontal.append(newLine)
				else:
					vertical.append(newLine)
		else: 
			for line in lines:
				# cv2.line(output, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0,255,0),2)
				[[x1,y1,x2,y2]] = line
				newLine = LineClass(x1,y1,x2,y2)
				if newLine.orientation == 'horizontal':
					horizontal.append(newLine)
				else:
					vertical.append(newLine)

		if debug:
			# Show image with lines drawn
			cv2.imshow("Lines",output)
			cv2.waitKey(0)
			
		return horizontal, vertical

	def findCorners (self, horizontal, vertical, colorEdges):
		'''
		Finds corners at intersection of horizontal and vertical lines.
		'''

		# Find corners (intersections of lines)
		corners = []
		for v in vertical:
			for h in horizontal:
				s1,s2 = v.find_intersection(h)
				corners.append([s1,s2])

		# remove duplicate corners
		dedupeCorners = []
		for c in corners:
			matchingFlag = False
			for d in dedupeCorners:
				if math.sqrt((d[0]-c[0])*(d[0]-c[0]) + (d[1]-c[1])*(d[1]-c[1])) < 30:
					matchingFlag = True
					break
			if not matchingFlag:
				dedupeCorners.append(c)

		for d in dedupeCorners:
			cv2.circle(colorEdges, (d[0],d[1]), 4, (0,0,255),2)


		if debug:
			#Show image with corners circled
			cv2.imshow("Corners",colorEdges)
			cv2.waitKey(0)

		return dedupeCorners

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

		letters = ['a','b','c','d','e','f','g','h']
		numbers = ['1','2','3','4','5','6','7','8']
		Squares = []
		
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
				Squares.append(newSquare)

		if not debug:
			#Show image with squares and ROI drawn and position labelled
			cv2.imshow("Squares", colorEdges)
			cv2.waitKey(0)

		return Squares

	def occupancySquares(self, image, Squares):

		image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		se=cv2.getStructuringElement(cv2.MORPH_RECT , (5,5))
		bg=cv2.morphologyEx(image, cv2.MORPH_DILATE, se, iterations=3)
		out_gray=cv2.divide(image, bg, scale=255)
		out_binary=cv2.threshold(out_gray, 100, 255, cv2.THRESH_OTSU )[1]

		kernel5 = np.ones((5,5),np.uint8)
		kernel3 = np.ones((3,3),np.uint8)
		erosion = cv2.erode(out_binary,kernel3,iterations = 2)

		# square = Squares[0]
		# roi = square.roi
		# print(roi)
		# print(erosion[roi[1]+109,roi[0]+375])

		occupancy_Squares = []
		for square in Squares:
			score = 0
			for dx in range(-12,13):
				for dy in range(-12,13):
					if erosion[square.roi[1]+dy, square.roi[0]+dx] == 0:
						score +=1
			if score > 0.6*500:
				occupancy_Squares.append(square)

		print([sq.position for sq in occupancy_Squares])

		cv2.imshow('erosion', erosion)
		cv2.waitKey(0)

		return occupancy_Squares

	def findPiece(self, image, occupancy_squares, trans_matrix):

		inv_maxtrix = np.linalg.pinv(trans_matrix)
		origin_points = []
		for square in occupancy_squares:
			perspective_point = np.array([[[square.roi[0],square.roi[1]]]],dtype=np.float32)
			origin_point = cv2.perspectiveTransform(perspective_point, inv_maxtrix)
			origin_point = origin_point.flatten()
			origin_point[0] = origin_point[0]+10
			origin_point[1] = origin_point[1]+10
			cv2.circle(image, (origin_point[0], origin_point[1]), 3, (0,0,255), 1)

		cv2.imshow('Piece Detection', image)
		cv2.waitKey(0)
		


image = cv2.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/frame_19.05/29.png")

board = board_Recognition(image)
result = board.initialize_Board()

# cv2.imshow("image",result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
