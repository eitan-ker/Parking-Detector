# this class meant for tracking objects that are inside parking area to minimizing the amount of total
# parking positions.
# cars that inside parking area will be tracked.
# save the positions of each car.
# when car stops moving we will return the max area size parking position.

import math


class Tracker:
    def __init__(self):
        self.center_points = {}  # id:(x,y)
        self.positions = {}  # id:[(x1,y1,w1,h1), (x2,y2,w2,h2)....]
        self.notMoving = {}  # id:(noMoveIterationCounter)
        self.optimalParkingPosition = {}  # id:[(x1,y1,w1,h1),(x2,y2,w2,h2)]
        self.id_count = 0

    # will save the largest parking in the first half and second half.
    # this is order to deal with getting out / getting in parking situations.
    def findOptimalParkingPosition(self, obj_id):
        positions = self.positions[obj_id]
        centerIndex = len(positions)//2
        # first half
        relevantPositions = positions[:centerIndex]
        optimalParkingPosition = [0, None]
        for pos in relevantPositions:
            area = pos[2] * pos[3]
            if area > optimalParkingPosition[0]:
                optimalParkingPosition[0] = area
                optimalParkingPosition[1] = pos
        self.optimalParkingPosition[obj_id].append(optimalParkingPosition[1])
        # second hald
        relevantPositions = positions[centerIndex:]
        optimalParkingPosition = [0, None]
        for pos in relevantPositions:
            area = pos[2] * pos[3]
            if area > optimalParkingPosition[0]:
                optimalParkingPosition[0] = area
                optimalParkingPosition[1] = pos
        self.optimalParkingPosition[obj_id].append(optimalParkingPosition[1])


    def assignValues(self, obj_id, x, y, w, h, cx, cy, ids_detected):
        self.center_points[obj_id] = (cx, cy)
        self.positions[obj_id].append([x, y, w, h])
        self.notMoving[obj_id] = 0
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
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                # same object detected
                if dist < 25:
                    self.assignValues(id, x, y, w, h, cx, cy, ids_detected)
                    same_object_detected = True
                    break

            # New object is detected we init all attributes
            if same_object_detected is False:
                self.positions[self.id_count] = []  # init this as list
                self.optimalParkingPosition[self.id_count] = []
                self.assignValues(self.id_count, x, y, w, h, cx, cy, ids_detected)
                self.id_count += 1

        # count number of iterations car hasn't been moving
        for id, pt in self.notMoving.items():
            if id not in ids_detected:
                self.notMoving[id] = pt + 1
            # X = 10, check if car hasn't been moving for X iterations and create optimalParking Position
            if self.notMoving[id] == 10:
                self.findOptimalParkingPosition(id)

    def getOptimalParkingPositions(self):
        if len(self.optimalParkingPosition) == 0:
            return []
        optimalParkingPosition = []
        tempOptimalParkingPosition = self.optimalParkingPosition.copy()
        for id, pt in tempOptimalParkingPosition.items():
            if len(pt) == 0:
                continue
            optimalParkingPosition.append(pt[0])
            optimalParkingPosition.append(pt[1])
            # once added the optimalParkingPosition, delete the ID obj from all attributes
            self.center_points.pop(id)
            self.positions.pop(id)
            self.notMoving.pop(id)
            self.optimalParkingPosition.pop(id)
        return optimalParkingPosition
