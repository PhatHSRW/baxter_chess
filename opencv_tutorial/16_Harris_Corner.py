import cv2
import numpy as np

img = cv2.resize(cv2.imread('/home/hsrw-robotics/PhatHuynh/python_test/opencv_tutorial/chessboard.jpg.png'),None,fx=0.5,fy=0.5)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)
def nothing(x):
    pass
cv2.namedWindow('Harris')
cv2.createTrackbar('BlockSize','Harris',1,29,nothing)
cv2.createTrackbar('kSize','Harris',1,29,nothing)
cv2.createTrackbar('k','Harris',1,10,nothing)

while(1):
    block = cv2.getTrackbarPos('BlockSize','Harris')
    ksize = cv2.getTrackbarPos('kSize','Harris')
    k = cv2.getTrackbarPos('k','Harris')

    if block == 0:
        block = 1
    elif block%2==0:
        block+1

    if ksize == 0:
        ksize = 1
    elif ksize%2==0:
        ksize = ksize+1


    dst = cv2.cornerHarris(gray,block,ksize,k/100)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    img[dst>0.01*dst.max()]=[0,0,255]

    cv2.imshow('dst',img)
    if cv2.waitKey(1) & 0xff == 27:
        cv2.destroyAllWindows()