import cv2
import numpy as np
import pickle
from shapely.geometry import Point, Polygon
from tracker import *
import threading
from parking_model import lock_posList

# MAYBE need to reArrange the point by (point1,point2,point3,point4) = (upper_left, upper_right,
# bottom_right, bottom_left)

# *****
# Init
# *****


# cap = cv2.VideoCapture('http://eitancamhome:eitancamhome@10.100.102.10:6677/video')
# cap = cv2.VideoCapture('resultvideoday1.avi')
cap = cv2.VideoCapture('resultvideoday3.avi')
# cap = cv2.VideoCapture('resultvideoday2.avi')
# cap = cv2.VideoCapture('resultvideomid.avi')


# Defined as (point1,point2,point3,point4) = (upper_left, upper_right, bottom_right, bottom_left). point = (width,height)
parkingArea = []
# Create moving tracker object
object_detector = cv2.createBackgroundSubtractorKNN()
# Create tracker object for parkingPositions
tracker = Tracker()

try:
    with open('parkingAreasPos', 'rb') as f:
        parkingAreas = pickle.load(f)
except:
    parkingAreas = []

# lock
with lock_posList:
    try:
        with open('parkingPositions3', 'rb') as f:
            parkingPositions = pickle.load(f)
    except:
        parkingPositions = []


# ********
# End Init
# ********


# *****************************
# Mark parking Area functions
# *****************************

# Arrange points clock wise direction
# def rearrangeParkingAreaPoint(parkingArea):


# Append to the parking area the point which was chosen using mouse
def chooseborderPoint(events, x, y, flags, params):
    if events == cv2.EVENT_LBUTTONDOWN:
        parkingArea.append((x, y))


# Mark 4 points of parking area
def markParkingArea(frame):
    global parkingArea
    while True:
        cv2.imshow("parkingMarker", frame)

        cv2.setMouseCallback("parkingMarker", chooseborderPoint)

        cv2.waitKey(1)

        if len(parkingArea) == 4:
            cv2.destroyWindow('parkingMarker')
            break

    # MAYBE need to reArrange the point by (point1,point2,point3,point4) = (upper_left, upper_right, bottom_right, bottom_left)
    # rearrangeParkingAreaPoint(parkingArea)

    parkingAreas.append(parkingArea)
    with open('parkingAreasPos', 'wb') as f:
        pickle.dump(parkingAreas, f)
    parkingArea = []


# Delete using mouse:
# Right click for parking area
# Left click for parking position
def deleteByClick(events, x, y, flags, params):
    if events == cv2.EVENT_RBUTTONDOWN:
        # check if click is in a parking area - delete it
        for i, pos in enumerate(parkingAreas):
            polyg = Polygon(pos)
            point = Point(x, y)
            if point.within(polyg):
                parkingAreas.pop(i)
                # save parking areas after deleting
                with open('parkingAreasPos', 'wb') as f:
                    pickle.dump(parkingAreas, f)
    if events == cv2.EVENT_LBUTTONDOWN:
        # check if click is in a parking area - delete it
        for i, pos in enumerate(parkingPositions):
            px, py, pw, ph = pos
            polyg = Polygon([(px, py), (px + pw, py), (px, py + ph), (px + pw, py + ph)])
            point = Point(x, y)
            if point.within(polyg):
                parkingPositions.pop(i)


# ****************************
# end marking Area functions
# ****************************


# if car is inside parking area, append it to list and save position
def addPotentialParkingPositions(x, y, w, h, potentialParkingPositions):
    # check if car is in area of parking, if tes add area to parkingPositions
    # check if point inside area
    for coord in parkingAreas:
        poly = Polygon(coord)
        p1 = Point(x, y)
        p2 = Point(x + w, y)
        p3 = Point(x, y + h)
        p4 = Point(x + w, y + h)
        if p1.within(poly) and p2.within(poly) and p3.within(poly) and p4.within(poly):
            potentialParkingPositions.append([x, y, w, h])


# Draw on main window all Parking areas
def drawParkingAreas(img):
    for pos in parkingAreas:
        size = len(pos)
        for i in range(size):
            cv2.line(img, pos[i % size], pos[(i + 1) % size], (255, 0, 255), 2)


# Draw Parking Positions
def drawParkingPositions(img):
    for pos in parkingPositions:
        cv2.rectangle(img, (pos[0], pos[1]), (pos[0] + pos[2], pos[1] + pos[3]), (0, 191, 255), 2)


# Parking positions detection algorithm
def detectionAlgorithm():
    while True:
        ret, frame = cap.read()
        view_frame = frame.copy()

        potentialParkingPositions = []

        imgBlur = cv2.GaussianBlur(frame, (51, 51), 2)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        mask = object_detector.apply(imgGray)
        _, mask = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        imgDilate = cv2.dilate(mask, kernel, iterations=2)

        edges = cv2.Canny(frame, 200, 200)

        drawParkingPositions(view_frame)
        drawParkingAreas(view_frame)

        contours, _ = cv2.findContours(imgDilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            (x, y, w, h) = cv2.boundingRect(cnt)
            if area > 2000:
                addPotentialParkingPositions(x, y, w, h, potentialParkingPositions)

                cv2.rectangle(view_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(view_frame, str(area), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        # tracker update and get optimal
        # from class tracker:
        # update
        tracker.update(potentialParkingPositions)
        # get optimal parking positions, objects will be deleted
        newParkingPositions = tracker.getOptimalParkingPositions()
        # append to parkingPositions
        if len(newParkingPositions) > 0:
            for pos in newParkingPositions:
                parkingPositions.append(pos)

        # lock
        with lock_posList:
            with open('parkingPositions', 'wb') as f:
                pickle.dump(parkingPositions, f)

        # cv2.imshow("original", frame)
        cv2.imshow("marked", view_frame)
        # cv2.imshow("masked", mask)
        cv2.imshow("imgDilate", imgDilate)
        # cv2.imshow("edges", edges)

        cv2.setMouseCallback("marked", deleteByClick)

        key = cv2.waitKey(1)
        if key == ord('p'):
            markParkingArea(frame)
        if key == ord('q'):
            break
