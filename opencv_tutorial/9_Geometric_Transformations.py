import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/Liv.jpg')


# -------------------Scaling------------------- #
''' Preferable interpolation methods are cv2.INTER_AREA for shrinking and cv2.INTER_CUBIC (slow) & cv2.INTER_LINEAR 
for zooming. By default, interpolation method used is cv2.INTER_LINEAR for all resizing purposes. '''

# res = cv2.resize(img,None,fx=1.5, fy=1.5, interpolation = cv2.INTER_CUBIC)

# -------------------Scaling------------------- #

#------------------Translation------------------#
''' transformation matrix M = [[1,0,tx] , [0,1,ty]]  '''

# rows,cols,_ = img.shape
# M = np.float32([[1,0,100],[0,1,50]])
# res = cv2.warpAffine(img,M,(cols,rows))
#------------------Translation------------------#


# ------------------Rotation------------------- #
''' modified transformation matrix M = [[alpha beta  (1-alpha)*cx-beta*cy],
                                        [-beta alpha beta*cx+(1-alpha*cy)]]
    where center=(cx,cy) | alpha=scale*cos(theta) | beta=scale*sin(theta) | theta: rotation angle'''

# rows,cols,_ = img.shape

# M = cv2.getRotationMatrix2D((cols/4,rows/2),30,1)    # center=(x/4,y/2), theta=30deg and scale=1
# res = cv2.warpAffine(img,M,(cols,rows))

# ------------------Rotation------------------- #


# ------------Affine Transformation------------ #
''' In affine transformation, all parallel lines in the original image will still be parallel in 
the output image.To find the transformation matrix, we need three points from input image and their 
corresponding locations in output image. Then cv2.getAffineTransform will create a 2x3 matrix which 
is to be passed to cv2.warpAffine.'''

# img = cv2.imread("/home/phathuynh/python_test/opencv_tutorial/tensorflow_logo.png")
# img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
# rows,cols,ch = img.shape

# pts1 = np.float32([[50,50],[200,50],[50,200]])
# pts2 = np.float32([[10,100],[200,50],[100,250]])

# M = cv2.getAffineTransform(pts1,pts2)

# dst = cv2.warpAffine(img,M,(cols,rows))

# plt.subplot(121),plt.imshow(img),plt.title('Input')
# plt.subplot(122),plt.imshow(dst),plt.title('Output')
# plt.show()

# ------------Affine Transformation------------ #


# ------------Perspective Transformation------------ #
img = cv2.imread("/home/phathuynh/python_test/opencv_tutorial/tensorflow_logo.png")
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
rows,cols,ch = img.shape

pts1 = np.float32([[56,65],[368,52],[28,387],[389,390]])
pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])

M = cv2.getPerspectiveTransform(pts1,pts2)

dst = cv2.warpPerspective(img,M,(300,300))

plt.subplot(121),plt.imshow(img),plt.title('Input')
plt.subplot(122),plt.imshow(dst),plt.title('Output')
plt.show()

# ------------Perspective Transformation------------ #



cv2.imshow("img",res)
cv2.waitKey(0)
cv2.destroyAllWindows()
