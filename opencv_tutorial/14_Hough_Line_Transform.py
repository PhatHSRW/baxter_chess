import cv2
import numpy as np

# ------------------Hough Tranform-------------------- #

# img = cv2.imread('baxter_cv/1620665544555344000.jpg')
# img = img[250:550,550:850]

# # cap = cv2.VideoCapture(0)
# # ret ,img = cap.read()
# lst = []

# def nothing(x):
#     pass
# cv2.namedWindow('Canny')
# cv2.createTrackbar('thresh_max','Canny',200,500,nothing)
# cv2.createTrackbar('thresh_min','Canny',50,500,nothing)

# cv2.namedWindow('Hough')
# cv2.createTrackbar('thresh','Hough',150,500,nothing)

# def draw_line(img):
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     img_re = np.zeros((gray.shape[0],gray.shape[1],3), np.uint8)  

#     thresh_hough = cv2.getTrackbarPos('thresh','Hough')

#     thresh_max = cv2.getTrackbarPos('thresh_max','Canny')
#     thresh_min = cv2.getTrackbarPos('thresh_min','Canny')

#     edges = cv2.Canny(gray,thresh_min,thresh_max,apertureSize = 3)
#     lines = cv2.HoughLines(edges,1,np.pi/180,thresh_hough)

#     # edges = cv2.Canny(gray, 50, 200, None, 3)
    
#     # # Copy edges to the images that will display the results in BGR
#     # cdst = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
#     # cdstP = np.copy(cdst)
#     # lines = cv2.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
    
#     for line in lines:
#         for rho,theta in line:
#             a = np.cos(theta)
#             b = np.sin(theta)
#             x0 = a*rho
#             y0 = b*rho
#             x1 = int(x0 + 1000*(-b))
#             y1 = int(y0 + 1000*(a))
#             x2 = int(x0 - 1000*(-b))
#             y2 = int(y0 - 1000*(a))
#             cv2.line(img_re,(x1,y1),(x2,y2),(0,0,255),2)
#             if [x1,y1,x2,y2] not in lst:
#                 lst.append([[x1,y1,x2,y2]])


#     cv2.imshow("img",img)
#     cv2.imshow('edges',edges)
#     cv2.imshow("re", img_re)



# while(1):
#     draw_line(img)
#     k = cv2.waitKey(1) & 0xFF
#     if k == 27 or k in [ord("q"), ord("e")]:
#         cv2.destroyAllWindows()
#         break

# print(len(lst))
# ------------------Hough Tranform-------------------- #



# # --------------Probabilistic Hough Tranform----------------- #

def nothing(x):
    pass
cv2.namedWindow('Canny')
cv2.createTrackbar('thresh_max','Canny',300,500,nothing)
cv2.createTrackbar('thresh_min','Canny',150,500,nothing)

cv2.namedWindow('Probabilistic Hough')
cv2.createTrackbar('thresh','Probabilistic Hough',100,500,nothing)
cv2.createTrackbar(' minLineLength','Probabilistic Hough',10,500,nothing)
cv2.createTrackbar('maxLineGap','Probabilistic Hough',20,500,nothing)

cap = cv2.VideoCapture(0)

while(1):
    img = cv2.resize(cv2.imread('/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/script/baxter_cv/frame_capture/1280x800_posC.jpg'),None, fx=0.7,fy=0.7)
    # _,img = cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_re = np.zeros((gray.shape[0],gray.shape[1],3), np.uint8) 

    maxLineGap = cv2.getTrackbarPos('maxLineGap','Probabilistic Hough')
    minLineLength = cv2.getTrackbarPos('minLineLength','Probabilistic Hough')
    thresh = cv2.getTrackbarPos('thresh','Probabilistic Hough')

    thresh_max = cv2.getTrackbarPos('thresh_max','Canny')
    thresh_min = cv2.getTrackbarPos('thresh_min','Canny')

    edges = cv2.Canny(gray,thresh_min,thresh_max,apertureSize = 3)

    lines = cv2.HoughLinesP(edges,1,np.pi/180,thresh, minLineLength, maxLineGap)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.line(img_re,(x1,y1),(x2,y2),(0,0,255),2)

    cv2.imshow("img",img)
    cv2.imshow('edges',edges)
    cv2.imshow("re", img_re)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k in [ord("q"), ord("e")]:
        break
    print(lines)
    print(lines.shape)

cv2.destroyAllWindows()

# # --------------Probabilistic Hough Tranform----------------- #

