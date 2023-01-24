import cv2
import pickle
import cvzone
import numpy as np
import time
import threading
from shapely.geometry import Point, Polygon


class Model:
    def __init__(self, stream, parkingPositionsPath):
        self.__outputFrame = None
        self.__lock = threading.Lock()
        self.__lock_posList = threading.Lock()
        self.__freeSpaces = 0
        self.__totalSpaces = 0
        self.__pixel_min = 80
        self.__cap = cv2.VideoCapture(stream)
        # lock
        with self.__lock_posList:
            try:
                with open(parkingPositionsPath, 'rb') as f:
                    self.__poslist = pickle.load(f)
            except:
                self.__poslist = []


    def __intersecting(self, position, positionList):
        if len(positionList) == 0:
            return False
        x, y, w, h = position[0], position[1], position[2], position[3]
        for pos in positionList:
            px, py, pw, ph = pos[0], pos[1], pos[2], pos[3]
            # first square
            # checking left upper corner
            if ((px <= x <= px + pw) and (py <= y <= py + ph) or (px <= x <= px + pw) and (py <= y + h <= py + ph) or
                    (px <= x + w <= px + pw) and (py <= y + h <= py + ph) or (px <= x <= px + pw) and (py <= y <= py + ph)):
                return True
            # second square
            if ((x <= px <= x + w) and (y <= py <= y + h) or (x <= px <= x + w) and (y <= py + ph <= y + h) or
                    (x <= px + pw <= x + w) and (y <= py + ph <= y + h) or (x <= px <= x + w) and (y <= py <= y + h)):
                return True
        return False


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
            # dp[i][0] = p2
            # dp[i][1] = l2
            dp[i] = [p2,l2]
        return dp[i]

    def __pixelCountArea(self, poslist, imgPro, countpixelsList):
        for pos in poslist:
            imgCrop = imgPro[pos[1]:pos[1] + pos[3], pos[0]:pos[0] + pos[2]]
            count = cv2.countNonZero(imgCrop)
            area = pos[2] * pos[3]
            countpixelsList.append([pos[0],pos[1],pos[2],pos[3],count, area])

    def __relevantPositions(self, countpixelsList, occupiedPositions, freeParkingPostiions):
        for i in range(len(countpixelsList)):
            # if self.__intersecting(countpixelsList[i], occupiedPositions) or \
            #         self.__intersecting(countpixelsList[i], freeParkingPostiions):
            #     continue
            # elif countpixelsList[i][4] < self.__pixel_min:
            #     # append to list of free spaces to later run algorithm to find optimal free positions and
            #     # append to parkingPositionList
            #     freeParkingPostiions.append(countpixelsList[i])
            # else:
            #     occupiedPositions.append(countpixelsList[i])
            if countpixelsList[i][4] < self.__pixel_min:
                # append to list of free spaces to later run algorithm to find optimal free positions and
                # append to parkingPositionList
                freeParkingPostiions.append(countpixelsList[i])
            else:
                occupiedPositions.append(countpixelsList[i])

    def __markFrames(self, parkingPositionList, imgPro, frame):
        for pos in parkingPositionList:
            x,y = pos[0], pos[1]

            imgCrop = imgPro[pos[1]:pos[1]+pos[3], pos[0]:pos[0]+pos[2]]
            # cv2.imshow(str(x*y), imgCrop)
            count = cv2.countNonZero(imgCrop)
            cvzone.putTextRect(frame, str(count), (pos[0]+(pos[2]//2),pos[1]+(pos[3]//2)), scale = 1, thickness = 1, offset = 0)

            # if less than 250 free -> color green else color red
            if count < self.__pixel_min:
                color = (0,255,0)
                thickness = 4

                # a lock might be needed for this
                self.__freeSpaces += 1
            else:
                color = (0,0,255)
                thickness = 2
            cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + pos[2], pos[1] + pos[3]), color, thickness)


    def __checkParkingSpace(self, imgPro, frame, poslist):
        self.__freeSpaces = 0
        # create modified posList
        # choose spaces that are occupied
        parkingPositionList = []
        occupiedPositions = []
        freeParkingPostiions = []
        countpixelsList = []
        # add pixel count to positions
        self.__pixelCountArea(poslist, imgPro, countpixelsList)

        # sort by pixel count index
        countpixelsList = sorted(countpixelsList, key=lambda x: (-x[4], x[5]))


        # ****************************************Delete*************************************************************
        # # add relevant positions
        # self.__relevantPositions(countpixelsList, occupiedPositions, freeParkingPostiions)
        # # run dynamic programming algorithm to choose max occupied parkings
        # dp_occ = [[-1, None]] * len(occupiedPositions)
        # numOfoccupiedSpaces, occupiedSpacePositions = self.__optimalFreePositions(0, occupiedPositions, [], dp_occ)
        # for pos in occupiedSpacePositions:
        #     parkingPositionList.append(pos)
        # ****************************************Delete*************************************************************

        # choose occupied positions from yolov5 model
        # choose the not intersecting (with occupied positions from yolov5 model) free parking positions
        # from pixelCountArea

        # run dynamic programming algorithm to choose max free parkings
        dp_free = [[-1, None]] * len(freeParkingPostiions)
        numOfFreeSpaces, freeSpacePositions = self.__optimalFreePositions(0, freeParkingPostiions, [], dp_free)
        for pos in freeSpacePositions:
            parkingPositionList.append(pos)

        # a lock might be needed for this
        self.__totalSpaces = len(parkingPositionList)

        # mark frames
        self.__markFrames(parkingPositionList, imgPro, frame)


    def getFreeSpaces(self):
        return self.__freeSpaces

    def getTotalSpaces(self):
        return self.__totalSpaces

    def getLockPosList(self):
        return self.__lock_posList


    # brighten the img. Was taken from:
    # https://stackoverflow.com/questions/44752240/how-to-remove-shadow-from-scanned-images-using-opencv
    def __no_shadows(self, img):
        rgb_planes = cv2.split(img)

        result_planes = []
        result_norm_planes = []
        for plane in rgb_planes:
            dilated_img = cv2.dilate(plane, np.ones((7, 7), np.uint8))
            bg_img = cv2.medianBlur(dilated_img, 21)
            diff_img = 255 - cv2.absdiff(plane, bg_img)
            norm_img = cv2.normalize(diff_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            result_planes.append(diff_img)
            result_norm_planes.append(norm_img)
        result_norm = cv2.merge(result_norm_planes)

        return result_norm


    def __proccess_frame(self, frame, poslist):

        no_shadow = self.__no_shadows(frame)

        # process img for testing in parking model
        imgGray = cv2.cvtColor(no_shadow, cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (25, 25), 1)
        kernel = np.ones((3, 3), np.uint8)
        imgDilate = cv2.dilate(imgBlur, kernel, iterations=1)
        imgThreshold = cv2.adaptiveThreshold(imgDilate, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 16)

        cv2.imshow("imgThreshold", imgThreshold)

        self.__checkParkingSpace(imgThreshold, frame, poslist)


    def stream(self):
        # imgDilate = None

        while True:
            # read posList from parkingPositions with lock
            with self.__lock_posList:
                try:
                    with open('parkingPositions', 'rb') as f:
                        poslist = pickle.load(f)
                except:
                    poslist = []

            if self.__cap.get(cv2.CAP_PROP_POS_FRAMES) == self.__cap.get(cv2.CAP_PROP_FRAME_COUNT):
                self.__cap.set(cv2.CAP_PROP_POS_FRAMES,0)

            success, frame = self.__cap.read()
            if success:
                self.__proccess_frame(frame, poslist)
                # frame = cv2.resize(frame, (1500, 850))
                with self.__lock:
                    self.__outputFrame = frame.copy()


            cv2.imshow("Video", frame)
            # cv2.imshow("imgThreshold", imgDilate)

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

