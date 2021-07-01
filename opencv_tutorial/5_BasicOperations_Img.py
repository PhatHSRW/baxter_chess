import cv2
import numpy as np
from matplotlib import pyplot as plt


# -----------------------------------------------------
# img = cv2.imread("/home/phathuynh/python_test/opencv_tutorial/Liv.jpg")
# px = img[120, 240]                      #pixel at position y=240, x=120
# print(px)
# img[20:100,50:130] = [0,0,0]            #change area y=[20:100], x=[50:130] to black
# print(img.dtype)

# cup = img[70:170,350:430]               #cup = region of image (ROI) (y,x)=[70:170,350:430]
# img[300:400,150:230] = cup              #ROI (y,x)=[300:400,150:230] is replaced by cup

# -----------------------------------------------------

# # -----------------------------------------------------
# img = np.zeros((512,512,3), np.uint8)
# print(img.item(100,150,2))
# for i in range(15):
#     img.itemset((100+i,150+i,2),255)
# # -----------------------------------------------------

# -----------------------------------------------------
# b,g,r = cv2.split(img)
# print(b,g)
# img[:,:,2]=0
# -----------------------------------------------------

# -------------Making borders (padding)----------------
BLUE = [255,0,0]

img1 = cv2.imread('/home/phathuynh/python_test/opencv_tutorial/opencv_logo.png')

replicate = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REPLICATE)
reflect = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REFLECT)
reflect101 = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REFLECT_101)
wrap = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_WRAP)
constant= cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_CONSTANT,value=BLUE)

plt.subplot(231),plt.imshow(img1,'gray'),plt.title('ORIGINAL')
plt.subplot(232),plt.imshow(replicate,'gray'),plt.title('REPLICATE')
plt.subplot(233),plt.imshow(reflect,'gray'),plt.title('REFLECT')
plt.subplot(234),plt.imshow(reflect101,'gray'),plt.title('REFLECT_101')
plt.subplot(235),plt.imshow(wrap,'gray'),plt.title('WRAP')
plt.subplot(236),plt.imshow(constant,'gray'),plt.title('CONSTANT')

plt.show()

# -------------Making borders (padding)----------------




# cv2.imshow("img", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()