import cv2
import numpy as np
from Line import Line
import math
import imutils


def initialize_mask(adaptiveThresh,img):
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
    cv2.drawContours(imgContours, [largest], -1, (0,0,0), 1)


    # Epsilon parameter needed to fit contour to polygon
    epsilon = 0.03 * Lperimeter
    # Approximates a polygon from chessboard edge
    chessboardEdge = cv2.approxPolyDP(largest, epsilon, True)

    # Create new all black image
    mask = np.zeros((img.shape[0], img.shape[1]), 'uint8')*125
    # Copy the chessboard edges as a filled white polygon size of chessboard edge
    cv2.fillConvexPoly(mask, chessboardEdge, 255, 1)
    # Assign all pixels that are white (i.e the polygon, i.e. the chessboard)
    extracted = np.zeros_like(img)
    extracted[mask == 255] = img[mask == 255]
    # remove strip around edge
    extracted[np.where((extracted == [125, 125, 125]).all(axis=2))] = [0, 0, 20]

    return extracted

def draw_line(img):

    horizontal = []
    vertical = []
    inter = []
    dedupeCorners = []

    # img_resize = img[250:650,450:850]
    img_resize = imutils.resize(img, width=800)
    gray = cv2.cvtColor(img_resize,cv2.COLOR_BGR2GRAY)

    adaptiveThresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 177, 2)

    mask = initialize_mask(adaptiveThresh,img_resize)

    thresh_max = cv2.getTrackbarPos('thresh_max','Canny')
    thresh_min = cv2.getTrackbarPos('thresh_min','Canny')

    edges = cv2.Canny(cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY),thresh_min,thresh_max,apertureSize = 3)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 20, np.array([]), 50, 10)

    for line in lines[0]:
        cv2.line(img_resize, (line[0],line[1]), (line[2],line[3]), (0,255,0),2)
        [x1,y1,x2,y2] = line
        print(line)
        line_detect = Line(x1,x2,y1,y2)

        if line_detect.orientation == 'horizontal':
            horizontal.append(line_detect)
        else: vertical.append(line_detect)
        for v in vertical:
            for h in horizontal:
                x,y = v.find_intersection(h)
                inter.append([x,y])

        # remove duplicate corners
        for c in inter:
            matchingFlag = False
            for d in dedupeCorners:
                if math.sqrt((d[0]-c[0])*(d[0]-c[0]) + (d[1]-c[1])*(d[1]-c[1])) < 20:
                    matchingFlag = True
                    break
            if not matchingFlag:
                dedupeCorners.append(c)

        for d in dedupeCorners:
            cv2.circle(img_resize, (d[0],d[1]), 10, (0,0,255))

    return img_resize, edges

    # print(dedupeCorners)


origin = cv2.imread('/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/chess_board/b1.jpg')

def nothing(x):
    pass
cv2.namedWindow('Canny')
cv2.createTrackbar('thresh_max','Canny',200,500,nothing)
cv2.createTrackbar('thresh_min','Canny',100,500,nothing)

img_resize, edges = draw_line(origin)

while(1):
    cv2.imshow("img",img_resize)
    cv2.imshow('edges',edges)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k in [ord("q"), ord("e")]:
        cv2.destroyAllWindows()
        break




