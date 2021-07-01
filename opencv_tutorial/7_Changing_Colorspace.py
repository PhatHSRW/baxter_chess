import cv2
import numpy as np

# flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
# print flags

cap = cv2.VideoCapture(0)

def callback(x):
    pass

cv2.namedWindow('controls',2)
cv2.resizeWindow("controls", 550,10)

cv2.createTrackbar('low H','controls',0,179,callback)
cv2.createTrackbar('high H','controls',179,179,callback)
 
cv2.createTrackbar('low S','controls',0,255,callback)
cv2.createTrackbar('high S','controls',255,255,callback)
 
cv2.createTrackbar('low V','controls',0,255,callback)
cv2.createTrackbar('high V','controls',255,255,callback)



while(1):

    H_low = cv2.getTrackbarPos('low H','controls')
    H_high = cv2.getTrackbarPos('high H','controls')
    S_low = cv2.getTrackbarPos('low S','controls')
    S_high = cv2.getTrackbarPos('high S','controls')
    V_low = cv2.getTrackbarPos('low V','controls')
    V_high = cv2.getTrackbarPos('high V','controls')

    # Take each frame
    _, frame = cap.read()
    frame = cv2.resize(frame,(0,0),fx=0.6,fy=0.6)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower = np.array([H_low,S_low,V_low])
    upper = np.array([H_high,S_high,V_high])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27 or k == ord("e"):
        break


cv2.destroyAllWindows()