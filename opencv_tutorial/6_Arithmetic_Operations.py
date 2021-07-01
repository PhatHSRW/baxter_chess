import cv2
import numpy as np

'''
There is a difference between OpenCV addition and Numpy addition. 
OpenCV addition is a saturated operation while Numpy addition is a modulo operation.
'''

# x = np.uint8([250])
# y = np.uint8([10])
# print cv2.add(x,y)      # 250+10 = 260 => 255
# print x+y               # 250+10 = 260 % 256 = 4



# # --------------Image Blending-------------------
# img1 = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/opencv_logo.png')
# img2 = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/tensorflow_logo.png')
# for i in range(2):
#     if img1.shape[i] > img2.shape[i]:
#         img1 = cv2.resize(img1, (img2.shape[1],img2.shape[0]))
#     else: img2 = cv2.resize(img2, (img1.shape[1],img1.shape[0]))

# print(img1.shape, img2.shape)
# for i in np.linspace(0,1,11):
#     print(i)
#     dst = cv2.addWeighted(img1,1-i,img2,i,2)

#     cv2.imshow('dst',dst)
#     cv2.waitKey(1000)
#     cv2.destroyAllWindows()

# # --------------Image Blending-------------------

img1 = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/Liv.jpg')
img2 = cv2.resize(cv2.imread('/home/phathuynh/python_test/opencv_tutorial/opencv_logo.png'),(0,0),fx=0.2,fy=0.2)
# I want to put logo on top-left corner, So I create a ROI
rows,cols,channels = img2.shape
roi = img1[50:rows+50, 100:cols+100 ]

# Now create a mask of logo and create its inverse mask also
img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)

cv2.imshow("gray",img2gray)
cv2.imshow("mask",mask)
cv2.imshow("mask_inv",mask_inv)

# Now black-out the area of logo in ROI
img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

# Take only region of logo from logo image.
img2_fg = cv2.bitwise_and(img2,img2,mask = mask)

# Put logo in ROI and modify the main image
dst = cv2.add(img1_bg,img2_fg)
img1[50:rows+50, 100:cols+100 ] = dst

cv2.imshow("bg",img1_bg)
cv2.imshow("fg",img2_fg)
cv2.imshow('res',img1)
cv2.waitKey(0)
cv2.destroyAllWindows()

