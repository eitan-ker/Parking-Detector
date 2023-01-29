import cv2
import pickle
import threading
from shapely.geometry import Point, Polygon
import torch.hub
import pandas


class Model:
    def __init__(self, stream, db, weights):
        self.__model = torch.hub.load('ultralytics/yolov5', weights, pretrained=True)
        self.__outputFrame = None
        self.__lock = threading.Lock()
        self.__freeSpaces = 0
        self.__totalSpaces = 0
        self.__pixel_min = 80
        self.__cap = cv2.VideoCapture(stream)
        self.__db = db
        self.__parkingAreas = self.__db.getParkingArea()
        self.__poslist = self.__db.getParkingPositions()


    def __intersecting(self, position, positionList):
        if len(positionList) == 0:
            return False
        x, y, w, h = position[0], position[1], position[2], position[3]
        for pos in positionList:
            px, py, pw, ph = pos[0], pos[1], pos[2], pos[3]

            # If one rectangle is on left side of other
            if x > px + pw or px > x + w:
                continue
            # If one rectangle is above other
            if y > py + ph or py > y + h:
                continue
            return True


    def __optimalFreePositions(self, i, freeParkingPostiions, l, dp):
        if i == len(freeParkingPostiions):
            return (0, l)

        if dp[i][0] != -1:
            return dp[i]

        p1, k1 = self.__optimalFreePositions(i+1, freeParkingPostiions, l, dp)
        if self.__intersecting(freeParkingPostiions[i], k1):
            dp[i] = [p1, k1]
            return dp[i]
        else:
            l2 = k1.copy()
            l2.append(freeParkingPostiions[i])
            p2, k2 = self.__optimalFreePositions(i+1, freeParkingPostiions, l2, dp)
            p2 += 1

        if p1 > p2:
            dp[i] = [p1,k1]
        else:
            dp[i] = [p2,l2]
        return dp[i]

    def __pixelCountArea(self, imgPro, countpixelsList):
        for pos in self.__poslist:
            imgCrop = imgPro[pos[1]:pos[1] + pos[3], pos[0]:pos[0] + pos[2]]
            count = cv2.countNonZero(imgCrop)
            area = pos[2] * pos[3]
            countpixelsList.append([pos[0], pos[1], pos[2], pos[3], count, area])


    def __relevantFreePositions(self, countpixelsList, parkingPositionList, freeParkingPostiions):
        for i in range(len(countpixelsList)):
            if self.__intersecting(countpixelsList[i], parkingPositionList):
                continue
            else:
                freeParkingPostiions.append(countpixelsList[i])

    def __getOccupiedPositions(self, occupied, occupiedPositions):
        for index, row in occupied.pandas().xyxy[0].iterrows():
            x1 = int(row['xmin'])
            y1 = int(row['ymin'])
            xn = int(row['xmax'])
            yn = int(row['ymax'])
            name = str(row['name'])
            if 'car' in name or 'truck' in name:

                occupiedPositions.append((x1, y1, xn-x1, yn-y1, 1))


    def __markFrames(self, parkingPositionList, imgPro, frame):
        freeSpaces = 0
        for pos in parkingPositionList:
            x,y = pos[0], pos[1]

            imgCrop = imgPro[pos[1]:pos[1]+pos[3], pos[0]:pos[0]+pos[2]]

            # if less than 250 free -> color green else color red
            if pos[4] == 0:
                color = (0,255,0)
                thickness = 4

                # a lock might be needed for this
                freeSpaces += 1
            else:
                color = (0,0,255)
                thickness = 2
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + pos[2], pos[1] + pos[3]), color, thickness)
        self.__freeSpaces = freeSpaces

    def __relevantOccupiedPositions(self, occupiedPositions, parkingPositionList):
        for pos in occupiedPositions:
            x, y, w, h, o = pos[0], pos[1], pos[2], pos[3], pos[4]
            self.__addPotentialParkingPositions(x, y, w, h, o, parkingPositionList)


    def __addPotentialParkingPositions(self, x, y, w, h, o, parkingPositionList):
        # check if car is in area of parking, if tes add area to parkingPositions
        # check if point ipo*96852
        # *nside area
        for coord in self.__parkingAreas:
            poly = Polygon(coord)
            p1 = Point(x, y)
            p2 = Point(x + w, y)
            p3 = Point(x, y + h)
            p4 = Point(x + w, y + h)
            if p1.within(poly) and p2.within(poly) and p3.within(poly) and p4.within(poly) and \
                    [x, y, w, h, o] not in parkingPositionList:
                parkingPositionList.append([x, y, w, h, o])


    def __checkParkingSpace(self, imgPro, frame):
        # create modified posList
        # choose spaces that are occupied
        parkingPositionList = []
        occupiedPositions = []
        freeParkingPostiions = []
        countpixelsList = []
        # add pixel count to positions
        self.__pixelCountArea(imgPro, countpixelsList)

        # sort by pixel count index
        countpixelsList = sorted(countpixelsList, key=lambda x: (-x[4], x[5]))

        # get all occupied positions by cars using yolov5
        occupied = self.__model(imgPro)
        self.__getOccupiedPositions(occupied, occupiedPositions)

        self.__relevantOccupiedPositions(occupiedPositions, parkingPositionList)

        # check all countpixelsList if intersecting with occupied - if not add to freeParkingPostiions
        self.__relevantFreePositions(countpixelsList, parkingPositionList, freeParkingPostiions)

        # choose the not intersecting (with occupied positions from yolov5 model) free parking positions
        # run dynamic programming algorithm to choose max free parkings
        dp_free = [[-1, None]] * len(freeParkingPostiions)
        numOfFreeSpaces, freeSpacePositions = self.__optimalFreePositions(0, freeParkingPostiions, [], dp_free)
        for pos in freeSpacePositions:
            parkingPositionList.append((pos[0], pos[1], pos[2], pos[3], 0))

        # a lock might be needed for this
        self.__totalSpaces = len(parkingPositionList)

        # mark frames
        self.__markFrames(parkingPositionList, imgPro, frame)


    def getFreeSpaces(self):
        return self.__freeSpaces

    def getTotalSpaces(self):
        return self.__totalSpaces


    def __proccess_frame(self, frame):

        # process img for testing in parking model
        imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (51, 51), 1)

        self.__checkParkingSpace(imgBlur, frame)


    def stream(self):
        # imgDilate = None

        while True:
            success, frame = self.__cap.read()

            if success:
                self.__poslist = self.__db.getParkingPositions()
                self.__proccess_frame(frame)
                # frame = cv2.resize(frame, (1500, 850))
                with self.__lock:
                    self.__outputFrame = frame.copy()

            # cv2.imshow("Video", frame)

            k = cv2.waitKey(1)
            if k == ord('q'):
                exit()

    def generate(self):

        # loop over frames from the output stream
        while True:
            # wait until the lock is acquired
            with self.__lock:
                # check if the output frame is available, otherwise skip
                # the iteration of the loop
                if self.__outputFrame is None:
                    continue

                # encode the frame in JPEG format
                (flag, encodedImage) = cv2.imencode(".jpg", self.__outputFrame)

                # ensure the frame was successfully encoded
                if not flag:
                    continue

            # yield the output frame in the byte format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

