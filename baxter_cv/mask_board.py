import math
from Line import LineClass
from Square import SquareClass
from Board import BoardClass
import math
import cv2
import numpy as np
import imutils

debug = False


class BoardDetection:

    def __init__(self, image):
        self.image = image

    def initialize_Board(self):

        # Binarize the photo
        adaptiveThresh,img = self.clean_Image(self.image)

        # Black out all pixels outside the border of the chessboard
        mask, self.points = self.initialize_mask(adaptiveThresh,img)

        perspective_output, trans_matrix = self.perspective(img, self.points)

        edges,colorEdges = self.find_edges(perspective_output)

    def clean_Image(self,image):
        '''
        Resizes and converts the photo to black and white for simpler analysis
        '''
        # resize image
        # img = image[200:700,400:900]
        img = imutils.resize(image, width=1000)
        # img = cv2.GaussianBlur(img,(5,5),0)
        
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Setting all pixels above the threshold value to white and those below to black
        # Adaptive thresholding is used to combat differences of illumination in the picture
        adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)
        # ret,adaptiveThresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        if debug:
            # Show thresholded image
            cv2.imshow("Adaptive Thresholding", adaptiveThresh)
            cv2.waitKey(0)

        return adaptiveThresh,img

    def initialize_mask(self,adaptiveThresh,img):
        '''
        Finds border of chessboard and blacks out all unneeded pixels
        '''

        # Find contours (closed polygons)
        contours, hierarchy = cv2.findContours(adaptiveThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
            if c == 0:
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
        cv2.drawContours(imgContours, [largest], -1, (0,255,0), 2)
        print(len(largest))
        if debug:
            # Show image with contours drawn
            cv2.imshow("Chess Board Contour",imgContours)
            cv2.waitKey(1)

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
        # remove strip around edge
        # extracted[np.where((extracted == [125, 125, 125]).all(axis=2))] = [0, 0, 20]


        if debug:
            # Show image with mask drawn
            cv2.imshow("mask",extracted)
            cv2.waitKey(1)
        return extracted, chessboardEdge

    def perspective(self,image, points):
        pts1 = np.float32(points)
        pts2 = np.float32([[0,0],[0,500],[500,500],[500,0]])
        Matrix = cv2.getPerspectiveTransform(pts1,pts2)

        # print(Matrix)
        transform = cv2.warpPerspective(image,Matrix,(500,500))
        # output = cv2.Canny(transform,100,120,apertureSize=3)
        

        transform = cv2.GaussianBlur(transform,(5,5),0)
        output = cv2.cvtColor(transform, cv2.COLOR_BGR2GRAY)
        output = cv2.equalizeHist(output)
        # output = cv2.Canny(output,50,200,apertureSize=3)
        

        if debug:
            # Show image with mask drawn
            cv2.imshow("transform",transform)
            cv2.imshow("transform_edge",output)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("s"):
                cv2.imwrite("chess_board_img/perspective.jpg",output)
        return output, Matrix

    def find_edges(self, image):
        edges = cv2.Canny(image, 50, 200, None, 3)
        colorEdges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)

        if not debug:
            cv2.imshow("Canny", edges)
            cv2.imshow("Canny_color", colorEdges)
            cv2.waitKey(1)

        
        return edges, colorEdges

# image = cv2.imread('/home/phathuynh/ws_baxter/src/baxter_thesis/script/DE3-ROB1-CHESS-master/perception/chessboardImages/Board.jpg')
# image = cv2.imread('/home/phathuynh/ws_baxter/src/baxter_thesis/src/chess_board/b3.jpg')
# board_detection = BoardDetection(image)

# board_detection.initialize_Board()




