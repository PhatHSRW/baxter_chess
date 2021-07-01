import cv2
import numpy as np

# -------------image loading--------------
# img = cv2.imread("/home/phathuynh/python_test/opencv_tutorial/Liv.jpg",cv2.IMREAD_GRAYSCALE)

# print(img.shape)
# img = cv2.resize(img,(700,300))
# print(img.shape)

# cv2.imshow("img", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# -------------image loading--------------


# -------------video loading--------------
cap = cv2.VideoCapture(0)
# ret = cap.set(3,640)
# ret = cap.set(4,480)
# print(cap.read()[1].shape)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

# -------------video loading--------------
