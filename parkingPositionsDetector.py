import cv2
import numpy as np
import pickle
from shapely.geometry import Point, Polygon
from tracker import *


class Detector:
    def __init__(self, stream, parkingPositionsPath, parkingAreaPath, lock_posList):
        self.__cap = cv2.VideoCapture(stream)
        self.__parkingArea = []
        self.__object_detector = cv2.createBackgroundSubtractorKNN()
        self.__tracker = Tracker()
        self.__lock_posList = lock_posList
        try:
            with open(parkingAreaPath, 'rb') as f:
                self.__parkingAreas = pickle.load(f)
        except:
            self.__parkingAreas = []

        # lock
        with self.__lock_posList:
            try:
                with open(parkingPositionsPath, 'rb') as f:
                    self.__parkingPositions = pickle.load(f)
            except:
                self.__parkingPositions = []



    # *****************************
    # Mark parking Area functions
    # *****************************

    # Arrange points clock wise direction
    # def rearrangeParkingAreaPoint(parkingArea):


    # Append to the parking area the point which was chosen using mouse
    def __chooseborderPoint(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            self.__parkingArea.append((x, y))


    # Mark 4 points of parking area
    def __markParkingArea(self, frame):
        while True:
            cv2.imshow("parkingMarker", frame)

            cv2.setMouseCallback("parkingMarker", self.__chooseborderPoint)

            cv2.waitKey(1)

            if len(self.__parkingArea) == 4:
                cv2.destroyWindow('parkingMarker')
                break

        # MAYBE need to reArrange the point by (point1,point2,point3,point4) = (upper_left, upper_right, bottom_right, bottom_left)
        # rearrangeParkingAreaPoint(parkingArea)

        self.__parkingAreas.append(self.__parkingArea)
        with open('parkingAreasPos', 'wb') as f:
            pickle.dump(self.__parkingAreas, f)
        self.__parkingArea = []


    # Delete using mouse:
    # Right click for parking area
    # Left click for parking position
    def __deleteByClick(self, events, x, y, flags, params):
        if events == cv2.EVENT_RBUTTONDOWN:
            # check if click is in a parking area - delete it
            for i, pos in enumerate(self.__parkingAreas):
                polyg = Polygon(pos)
                point = Point(x, y)
                if point.within(polyg):
                    self.__parkingAreas.pop(i)
                    # save parking areas after deleting
                    with open('parkingAreasPos', 'wb') as f:
                        pickle.dump(self.__parkingAreas, f)
        if events == cv2.EVENT_LBUTTONDOWN:
            # check if click is in a parking area - delete it
            for i, pos in enumerate(self.__parkingPositions):
                px, py, pw, ph = pos
                polyg = Polygon([(px, py), (px + pw, py), (px, py + ph), (px + pw, py + ph)])
                point = Point(x, y)
                if point.within(polyg):
                    self.__parkingPositions.pop(i)


    # ****************************
    # end marking Area functions
    # ****************************


    # if car is inside parking area, append it to list and save position
    def __addPotentialParkingPositions(self, x, y, w, h, potentialParkingPositions):
        # check if car is in area of parking, if tes add area to parkingPositions
        # check if point inside area
        for coord in self.__parkingAreas:
            poly = Polygon(coord)
            p1 = Point(x, y)
            p2 = Point(x + w, y)
            p3 = Point(x, y + h)
            p4 = Point(x + w, y + h)
            if p1.within(poly) and p2.within(poly) and p3.within(poly) and p4.within(poly):
                potentialParkingPositions.append([x, y, w, h])


    # Draw on main window all Parking areas
    def __drawParkingAreas(self, img):
        for pos in self.__parkingAreas:
            size = len(pos)
            for i in range(size):
                cv2.line(img, pos[i % size], pos[(i + 1) % size], (255, 0, 255), 2)


    # Draw Parking Positions
    def __drawParkingPositions(self, img):
        for pos in self.__parkingPositions:
            cv2.rectangle(img, (pos[0], pos[1]), (pos[0] + pos[2], pos[1] + pos[3]), (0, 191, 255), 2)


    # Parking positions detection algorithm
    def detectionAlgorithm(self):
        while True:
            ret, frame = self.__cap.read()
            view_frame = frame.copy()

            potentialParkingPositions = []

            imgBlur = cv2.GaussianBlur(frame, (51, 51), 2)
            imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
            mask = self.__object_detector.apply(imgGray)
            _, mask = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
            kernel = np.ones((5, 5), np.uint8)
            imgDilate = cv2.dilate(mask, kernel, iterations=2)


            self.__drawParkingPositions(view_frame)
            self.__drawParkingAreas(view_frame)

            contours, _ = cv2.findContours(imgDilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                (x, y, w, h) = cv2.boundingRect(cnt)
                if area > 1400:
                    self.__addPotentialParkingPositions(x, y, w, h, potentialParkingPositions)

                    cv2.rectangle(view_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(view_frame, str(area), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            # tracker update and get optimal
            # from class tracker:
            # update
            self.__tracker.update(potentialParkingPositions)
            # get optimal parking positions, objects will be deleted
            newParkingPositions = self.__tracker.getOptimalParkingPositions()
            # append to parkingPositions
            if len(newParkingPositions) > 0:
                for pos in newParkingPositions:
                    self.__parkingPositions.append(pos)

            # lock
            with self.__lock_posList:
                with open('parkingPositions', 'wb') as f:
                    pickle.dump(self.__parkingPositions, f)

            cv2.imshow("marked", view_frame)
            cv2.imshow("imgDilate", imgDilate)
            #
            cv2.setMouseCallback("marked", self.__deleteByClick)

            key = cv2.waitKey(1)
            if key == ord('p'):
                self.__markParkingArea(frame)
            if key == ord('q'):
                break
