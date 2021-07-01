import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/Vy.jpg',0)
img = cv2.resize(img,None,fx =0.5,fy=0.5)
cap = cv2.VideoCapture(1)

def nothing(x):
    pass
cv2.namedWindow('control')
cv2.createTrackbar('Max','control',0,500,nothing)
cv2.createTrackbar('Min','control',0,500,nothing)


while(1):
    _, frame = cap.read()
    frame = cv2.resize(frame,(0,0),fx=0.6,fy=0.6)
    
    max_value = cv2.getTrackbarPos('Max','control')
    min_value = cv2.getTrackbarPos('Min','control')

    edges = cv2.Canny(frame,min_value,max_value)


    cv2.imshow("img",img)
    cv2.imshow('edges',edges)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k in [ord("q"), ord("e")]:
        break



cv2.destroyAllWindows(0)