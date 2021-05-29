import cv2


image = cv2.imread("/home/phathuynh/Desktop/transform_screenshot_2.png")

lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

lab_planes = cv2.split(lab)

clahe = cv2.createCLAHE(clipLimit=2.0,tileGridSize=(9,9))

lab_planes[0] = clahe.apply(lab_planes[0])

lab = cv2.merge(lab_planes)

bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

cv2.imshow("ouput",bgr)
cv2.imshow("input", image)
cv2.waitKey(0)
cv2.destroyAllWindows()