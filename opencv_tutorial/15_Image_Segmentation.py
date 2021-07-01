import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/coins.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

def nothing(x):
    pass

cv2.namedWindow("control")
cv2.createTrackbar("thresh_min", "control", 0, 255, nothing)
cv2.createTrackbar("thresh_max", "control", 0, 255, nothing)

while(1):
    img = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/coins.png')
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    thresh_min = cv2.getTrackbarPos("thresh_min", "control")
    thresh_max= cv2.getTrackbarPos("thresh_max", "control")

    ret, thresh = cv2.threshold(gray,thresh_min,thresh_max,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    # noise removal
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

    # sure background area
    sure_bg = cv2.dilate(opening,kernel,iterations=3)

    # Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)

    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)

    # Marker labelling
    ret, markers = cv2.connectedComponents(sure_fg)

    # Add one to all labels so that sure background is not 0, but 1
    markers = markers+1

    # Now, mark the region of unknown with zero
    markers[unknown==255] = 0

    markers = cv2.watershed(img,markers)
    img[markers == -1] = [255,0,0]  

    cv2.imshow("origin", img)
    cv2.imshow("thresh",thresh)
    cv2.imshow("background", sure_bg)
    cv2.imshow("frontground", sure_fg)

    k = cv2.waitKey(1)
    if k == 27 or k == ord("q"):
        break

cv2.destroyAllWindows()