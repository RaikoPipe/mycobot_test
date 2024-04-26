import cv2
import cv2.aruco as aruco
import numpy as np
import math

#instantiate aruco detecter
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(dictionary, parameters)

def cap_frame():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Define the starting point of the crop (top-left corner)
    start_x = 150  # example value
    start_y = 60 # example value

    # Define the size of the crop
    crop_size = 350
    # crop the image
    frame = frame[start_y:start_y + crop_size, start_x:start_x + crop_size]

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
    return frame


def shape_detect(frame):
    """shape recognition"""
    # keep the original image
    img = frame.copy()
    x = 0
    y = 0

    Alpha = 65.6
    Gamma = -8191.5
    cal = cv2.addWeighted(img, Alpha, img, 0, Gamma)
    # 转换为灰度图片
    gray = cv2.cvtColor(cal, cv2.COLOR_BGR2GRAY)

    # a etching operation on a picture to remove edge roughness
    erosion = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=2)

    # the image for expansion operation, its role is to deepen the color depth in the picture
    dilation = cv2.dilate(erosion, np.ones(
        (1, 1), np.uint8), iterations=2)

    # 设定灰度图的阈值 175, 255
    _, threshold = cv2.threshold(dilation, 175, 255, cv2.THRESH_BINARY)
    # 边缘检测
    edges = cv2.Canny(threshold, 50, 100)
    # 检测物体边框
    contours, _ = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        for cnt in contours:
            # if 6000>cv2.contourArea(cnt) and cv2.contourArea(cnt)>4500:
            print(cv2.contourArea(cnt))
            if cv2.contourArea(cnt) > 15:
                objectType = None
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                objCor = len(approx)

                boxes = [
                    box
                    for box in [cv2.boundingRect(c) for c in contours]
                    if min(img.shape[0], img.shape[1]) / 10
                       < min(box[2], box[3])
                       < min(img.shape[0], img.shape[1]) / 1
                ]

                for box in boxes:
                    x, y, w, h = box
                # find the largest object that fits the requirements
                c = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img, [box], 0, (153, 153, 0), 2)
                x = int(rect[0][0])
                y = int(rect[0][1])
                rot = rect[2]

                if objCor == 3:
                    objectType = ["Triangle", "三角形"]
                    # cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)

                elif objCor == 4:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                    _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                    aspRatio = _W / float(_H)
                    if 0.98 < aspRatio < 1.03:
                        objectType = ["Square", "正方形"]
                        cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 3)
                    else:
                        objectType = ["Rectangle", "长方形"]
                        cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 3)

                elif objCor >= 5:
                    objectType = ["Circle", "圆形"]

                    # cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                else:
                    pass

    if abs(x) + abs(y) > 0:
        return x, y, rot
    else:
        return None


def aruco_detect(frame):
    # Detect ArUco markers
    corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(frame)

    xyrot = []

    if ids is not None:
        # Estimate pose of each marker
        for i in range(len(ids)):
            corner = corners[i][0]
            center = corner.mean(axis=0)
            angle = np.arctan2(corner[0][1] - corner[2][1], corner[0][0] - corner[2][0]) * 180 / np.pi
            xyrot.append([int(center[0]), int(center[1]), angle])

            print(
                f"Marker ID: {ids[i][0]}, Center: ({int(center[0])}, {int(center[1])}), Rotation: {angle:.2f} degrees")

    return xyrot, ids[0]
