import numpy as np
import torch.hub
import yolov5
import cv2

cap = cv2.VideoCapture("resultvideo2013.avi")
model = torch.hub.load('ultralytics/yolov5', 'yolov5x', pretrained=True)

while True:

    success, frame = cap.read()

    imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (51, 51), 1)

    result = model(imgBlur)
    frame = np.squeeze(result.render())

    cv2.imshow("Video", frame)

    k = cv2.waitKey(1)
    if k == ord('q'):
        exit()