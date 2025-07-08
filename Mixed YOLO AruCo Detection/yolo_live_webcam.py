
# --- Final Live Webcam YOLOv8 Object Detection Script with Console Output ---

from ultralytics import YOLO
import cv2
import math

import socket
import random
from time import sleep

import cv2 as cv
from cv2 import aruco
import numpy as np
from datetime import datetime

# camera matrix, result from camera calibration
mtx = np.array([[1.38631135e+03, 0.00000000e+00, 9.47574169e+02],
                [0.00000000e+00, 1.38595261e+03, 5.32637110e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# deistortion coefficients, result from camera calibration
dst = np.array([[ 2.12306762e-01, -1.11055012e+00,  4.91556115e-03, -1.61109539e-04, 1.71179577e+00]])

# set coordinate system
marker_length = 0.055 # unit: meters
obj_points = np.array([[-marker_length/2, marker_length/2, 0],
                       [marker_length/2, marker_length/2, 0],
                       [marker_length/2, -marker_length/2, 0],
                       [-marker_length/2, -marker_length/2, 0]], dtype=np.float32).reshape(-1, 1, 3)

# 'DICT_4X4_50' dictionary: a marker dictionary with 4x4 grid of markers (50 different markers)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Create detector parameters
detector_params = aruco.DetectorParameters()
# create a detector
detector = aruco.ArucoDetector(marker_dict, detector_params)

# Start webcam / video capture
# cap = cv2.VideoCapture(0) # Default web cam
# cap.set(3, 640)
# cap.set(4, 480)
cap = cv2.VideoCapture('markerTester.png') # Image test

# Load YOLOv8 model
model = YOLO("yolo-Weights/yolov8n.pt")

# Use model's built-in class names
classNames = model.names

# Socket information
ip = '10.0.0.130'
port = 1755

# send object data to unity socket
def sendYoloDataToSocket(objClassName, x1, y2, x2, y2):
    s = socket.socket()   
    s.connect((ip, port))
    s.send((
        str(objClassName) + ","+ 
        str(x) + ","+ 
        str(y) + ","+ 
        str(z) + ","+ 
        str(Rx) + ","+ 
        str(Ry) + ","+ 
        str(Rz)).encode())
    s.close()

def sendAruCoDataToSocket(markerCorner, tvec, rvec):
    s = socket.socket()
    s.connect((ip, port))
    s.send((
        str(markerCorner) + ","+
        str(tvec) + ","+
        str(rvec).encode()
    ))
    s.close()

while True:

    success, img = cap.read()
    # Do processing on img, draw displayed img.
    displayImg = img

    if success:
        # convert the frame into gray frame
        gray_frame = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # find all the aruco markers in the image
        marker_cornes, marker_ids, reject_candidates = detector.detectMarkers(gray_frame)
        if marker_ids is not None and len(marker_ids) > 0:
            aruco.drawDetectedMarkers(displayImg, marker_cornes, marker_ids)
            rvecs, tvecs = [], []
            for i in range(len(marker_cornes)):
                _, rvec, tvec = cv.solvePnP(obj_points, marker_cornes[i], mtx, dst)
                rvecs.append(rvec)
                tvecs.append(tvec)
            for i in range(len(marker_cornes)):
                # 0.1 is the axis length, here also can add the thickness after the axis length
                cv.drawFrameAxes(displayImg, mtx, dst, rvecs[i], tvecs[i], 0.1)
                # send AruCo data
                sendAruCoDataToSocket(marker_cornes[i], tvecs[i], rvecs[i])
        else:
            cv.putText(displayImg, "No id", (0,64), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255),2,cv.LINE_AA)
        key = cv.waitKey(1) # 1 means delay in 1 milliseconds
        if key == ord('q'): # input 'q' to break the loop (close camera)
            break
        elif key == ord('s'):  # input 's' to save the pic
            cv.imwrite(str(datetime.now().strftime("%Y%m%d_%H%M%S")) + ".jpg", displayImg)

    if not success:
        break

    # Perform inference
    results = model(img, stream=True)

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0].item()
            cls_id = int(box.cls[0])

            # Draw bounding box
            cv2.rectangle(displayImg, (x1, y1), (x2, y2), (255, 0, 255), 2)

            # Put label
            label = f"{classNames[cls_id]} {conf:.2f}"
            cv2.putText(displayImg, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Print live detection info
            print(f"Detected: {classNames[cls_id]} | Confidence: {conf:.2f} | Box: ({x1}, {y1}, {x2}, {y2})")
            sendYoloDataToSocket(classNames[cls_id], x1, y1, x2, y2)

    cv2.imshow("YOLOv8 Live Webcam Detection", displayImg)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
