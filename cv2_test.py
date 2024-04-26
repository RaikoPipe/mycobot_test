import cv2
import numpy as np
import math





# Define the starting point of the crop (top-left corner)
start_x = 165  # example value
start_y = 90  # example value

# Define the size of the crop
crop_size = 312

# capture the frame
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Capture frame-by-frame
ret, frame = cap.read()
if not ret:
    print("Can't receive frame (stream end?). Exiting ...")


# crop the image
frame = frame[start_y:start_y + crop_size, start_x:start_x + crop_size]

x,y, rot = shape_detect(frame)
print(x, y, rot)
cv2.imshow("img", frame)

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()







