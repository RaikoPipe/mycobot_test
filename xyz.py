import cv2

from cv2_rec import aruco_detect, cap_frame


frame = cap_frame()
cv2.imshow("img", frame)
x, y, z = aruco_detect(frame)[0]

cv2.waitKey(0)
cv2.destroyAllWindows()