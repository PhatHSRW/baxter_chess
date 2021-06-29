import cv2

image = cv2.imread('/home/hsrw-robotics/PhatHuynh/ws_baxter/src/baxter_thesis/src/close_to_piece/black_A5.png')
image = image[50:350,300:800]
filter = cv2.boxFilter(image,0,(3,3), normalize = True)
gray = cv2.cvtColor(filter, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray', gray)

se=cv2.getStructuringElement(cv2.MORPH_RECT , (3,3))
edge = cv2.Canny(gray,30,80)
bg=cv2.morphologyEx(edge, cv2.MORPH_DILATE, se, iterations=5)
cv2.circle(gray,((550, 125)),5,255,2)
cv2.imshow('result', bg)
cv2.imshow('edge', edge)

cv2.waitKey(0)
cv2.destroyAllWindows()