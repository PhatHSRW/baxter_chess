import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('/home/hsrw-robotics/PhatHuynh/python_test/opencv_tutorial/filter_sample.png')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

kernel = np.ones((5,5),np.float32)/25
averaging = cv2.filter2D(img,-1,kernel)

blur = cv2.blur(img,(5,5))

gauss = cv2.GaussianBlur(img,(5,5),0)

median = cv2.medianBlur(img,5)

bilateral = cv2.bilateralFilter(img,9,75,75)

plt.figure(figsize=(10,10))
plt.subplot(231),plt.imshow(img),plt.title('Original'), plt.xticks([]), plt.yticks([])
plt.subplot(232),plt.imshow(averaging),plt.title('Averaging'), plt.xticks([]), plt.yticks([])
plt.subplot(233),plt.imshow(blur),plt.title('Blur'), plt.xticks([]), plt.yticks([])
plt.subplot(234),plt.imshow(gauss),plt.title('Gaussian'), plt.xticks([]), plt.yticks([])
plt.subplot(235),plt.imshow(median),plt.title('Median'), plt.xticks([]), plt.yticks([])
plt.subplot(236),plt.imshow(bilateral),plt.title('Bilateral'), plt.xticks([]), plt.yticks([])

plt.show()

