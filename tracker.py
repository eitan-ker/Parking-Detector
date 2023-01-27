# this class meant for tracking objects that are inside parking area to minimizing the amount of total
# parking positions.
# cars that inside parking area will be tracked.
# save the positions of each car.
# when car stops moving we will return the max area size parking position.

import math


class Tracker:
    def __init__(self):
        self.__center_points = {}  # id:(x,y)
        self.__positions = {}  # id:[(x1,y1,w1,h1), (x2,y2,w2,h2)....]
        self.__notMoving = {}  # id:(noMoveIterationCounter)
        self.__optimalParkingPosition = {}  # id:[(x1,y1,w1,h1),(x2,y2,w2,h2)]
        self.__id_count = 0
        # this will deal with unwanted partings caused by shadows for example
        # id:legit - if id detected += 1, if detected > 2 then assign values in detected

    # will save the largest parking in the first half and second half.
    # this is order to deal with getting out / getting in parking situations.
    def __findOptimalParkingPosition(self, obj_id):
        positions = self.__positions[obj_id]
        centerIndex = len(positions)//2
        # first half
        relevantPositions = positions[:centerIndex]
        optimalParkingPosition = [0, None]
        for pos in relevantPositions:
            area = pos[2] * pos[3]
            if area > optimalParkingPosition[0]:
                optimalParkingPosition[0] = area
                optimalParkingPosition[1] = pos
        self.__optimalParkingPosition[obj_id].append(optimalParkingPosition[1])
        # second hald
        relevantPositions = positions[centerIndex:]
        optimalParkingPosition = [0, None]
        for pos in relevantPositions:
            area = pos[2] * pos[3]
            if area > optimalParkingPosition[0]:
                optimalParkingPosition[0] = area
                optimalParkingPosition[1] = pos
        self.__optimalParkingPosition[obj_id].append(optimalParkingPosition[1])


    def __assignValues(self, obj_id, x, y, w, h, cx, cy, ids_detected):
        self.__center_points[obj_id] = (cx, cy)
        self.__positions[obj_id].append([x, y, w, h])
        self.__notMoving[obj_id] = 0
        # new car with no id that has been detected
        ids_detected.append(obj_id)

    def update(self, objects_rect):
        ids_detected = []

        # Get center point of an object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.__center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                # same object detected
                if dist < 50:
                    self.__assignValues(id, x, y, w, h, cx, cy, ids_detected)
                    same_object_detected = True
                    break

            # New object is detected we init all attributes
            if same_object_detected is False:
                self.__positions[self.__id_count] = []  # init this as list
                self.__optimalParkingPosition[self.__id_count] = []
                self.__assignValues(self.__id_count, x, y, w, h, cx, cy, ids_detected)
                self.__id_count += 1

        # count number of iterations car hasn't been moving
        for id, pt in self.__notMoving.items():
            if id not in ids_detected:
                self.__notMoving[id] = pt + 1
            # X = 10, check if car hasn't been moving for X iterations and create optimalParking Position
            if self.__notMoving[id] == 10:
                self.__findOptimalParkingPosition(id)

    def getOptimalParkingPositions(self):
        if len(self.__optimalParkingPosition) == 0:
            return []
        optimalParkingPosition = []
        tempOptimalParkingPosition = self.__optimalParkingPosition.copy()
        for id, pt in tempOptimalParkingPosition.items():
            if len(pt) == 0:
                continue
            for p in pt:
                if p is not None:
                    optimalParkingPosition.append(p)
            # optimalParkingPosition.append(pt[0])
            # optimalParkingPosition.append(pt[1])
            # once added the optimalParkingPosition, delete the ID obj from all attributes
            self.__center_points.pop(id)
            self.__positions.pop(id)
            self.__notMoving.pop(id)
            self.__optimalParkingPosition.pop(id)
        return optimalParkingPosition
