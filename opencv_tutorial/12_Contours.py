import numpy as np
import cv2

origin = cv2.resize(cv2.imread('/home/phathuynh/python_test/opencv_tutorial/contour_test.png'),None,fx=1.5,fy=1.5)
imgray = cv2.cvtColor(origin,cv2.COLOR_BGR2GRAY)

ret,thresh = cv2.threshold(imgray,220,255,cv2.THRESH_BINARY_INV)
img_canny = cv2.Canny(imgray,30,200)
_,contours_thresh,hierarchy_thresh = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
_,contours_canny,hierarchy_canny = cv2.findContours(img_canny,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

re_thresh = np.copy(origin)
re_canny = np.copy(origin)
# contour_4 = contours[3]
# cv2.drawContours(re, [contour_4] ,0, (0,0,0),3)
# cv2.drawContours(re_thresh, contours_thresh ,-1, (0,0,0),5)
# cv2.drawContours(re_canny, contours_canny ,-1, (0,0,0),5)

screenCnt =[]
for c in contours_thresh:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02*peri, True)

    if len(approx) == 3:
		screenCnt.append(approx)
		

cv2.drawContours(re_canny, screenCnt, -1, (0, 0, 0), 3) 
cv2.imshow("shape detected", re_canny) 


# cv2.imshow("canny", img_canny)
# cv2.imshow("threshold", thresh)
# cv2.imshow("result_canny", re_canny)
# cv2.imshow("result_thresh", re_thresh)

cv2.waitKey(0)
cv2.destroyAllWindows()