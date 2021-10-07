import cv2
import numpy as np

while True:
    image = cv2.imread("/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/board_frame/current_frame.jpg")
    image = image[100:400,300:700]
    # image = image[150:600, 350:750]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.medianBlur(gray, 3)
    

    # th2 = cv2.adaptiveThreshold(gray,200,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
    # th3 = cv2.adaptiveThreshold(gray,200,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

    # se=cv2.getStructuringElement(cv2.MORPH_RECT , (3,3))
    # edge = cv2.Canny(img_blur,40,80)
    # bg=cv2.morphologyEx(edge, cv2.MORPH_DILATE, se, iterations=4)

    circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1, img_blur.shape[0]/5, param1=90, param2=12, minRadius=24, maxRadius=30)
    # Draw detected circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        print('circle', circles)
        for i in circles[0, :]:
            pass
            # Draw outer circle
            # cv2.circle(gray, (i[0], i[1]), i[2], 255, -1)
            # Draw inner circle
            # cv2.circle(gray, (i[0], i[1]), 2, (0, 0, 255), 3)

    # cv2.circle(gray,)
    cv2.imshow('gray', gray)
    # cv2.imshow('morpho', bg)
    # cv2.imshow('edge', edge)
    # cv2.imshow('thres2',  th2)
    # cv2.imshow('thres3',  th3)

    k = cv2.waitKey(1000)
    if k == ord('q'):
        break
        cv2.destroyAllWindows()
