import cv2
import numpy as np

img = np.ones((640,480), dtype=np.uint8)
print(img.shape)
cv2.imshow("img", img)

cv2.line(img,(50,20),(250,350),(255,0,0),5)

cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()