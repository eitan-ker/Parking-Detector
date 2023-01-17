import cv2
import pickle
import cvzone
import numpy as np
import time
import threading
from shapely.geometry import Point, Polygon

# video feed
# try:
#     with open('CarParkPos', 'rb') as f:
#         poslist = pickle.load(f)
# except:
#     poslist = []



outputFrame = None
lock = threading.Lock()
lock_posList = threading.Lock()


# cap = cv2.VideoCapture('parkinglot1.mp4')
# cap = cv2.VideoCapture('resultvideoday1.avi')
cap = cv2.VideoCapture('resultvideoday3.avi')
# cap = cv2.VideoCapture('resultvideoday2.avi')
# cap = cv2.VideoCapture('resultvideomid.avi')
# cap = cv2.VideoCapture('http://eitancamhome:eitancamhome@10.100.102.10:6677/video')


time.sleep(2.0)
freeSpaces = 0
totalSpaces = 0
pixel_min = 80

# lock
with lock_posList:
    try:
        with open('parkingPositions3', 'rb') as f:
            poslist = pickle.load(f)
    except:
        poslist = []


def intersecting(position, positionList):
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


def optimalFreePositions(i, freeParkingPostiions, l, dp):
    if i == len(freeParkingPostiions):
        return (0, l)

    if dp[i][0] != -1:
        return dp[i]

    p1, k1 = optimalFreePositions(i+1, freeParkingPostiions, l, dp)
    if intersecting(freeParkingPostiions[i], k1):
        dp[i] = [p1, k1]
        return dp[i]
    else:
        l2 = k1.copy()
        l2.append(freeParkingPostiions[i])
        p2, k2 = optimalFreePositions(i+1, freeParkingPostiions, l2, dp)
        p2 += 1

    if p1 > p2:
        dp[i] = [p1,k1]
    else:
        # dp[i][0] = p2
        # dp[i][1] = l2
        dp[i] = [p2,l2]
    return dp[i]


def checkParkingSpace(imgPro, frame, poslist):
    global freeSpace, totalSpaces
    freeSpace = 0

    # create modified posList
    # choose spaces that are occupied
    parkingPositionList = []
    freeParkingPostiions = []
    countpixelsList = []
    # add pixel count to positions
    for pos in poslist:
        imgCrop = imgPro[pos[1]:pos[1] + pos[3], pos[0]:pos[0] + pos[2]]
        count = cv2.countNonZero(imgCrop)
        area = pos[2] * pos[3]
        countpixelsList.append([pos[0],pos[1],pos[2],pos[3],count, area])
    # sort by pixel count index
    # countpixelsList.sort(key=lambda countpixelsList: countpixelsList[4])
    countpixelsList = sorted(countpixelsList, key=lambda x: (-x[4], x[5]))

    # add relevant positions
    for i in range(len(countpixelsList)):
        if intersecting(countpixelsList[i], parkingPositionList):
            continue
        elif countpixelsList[i][4] < pixel_min:
            # append to list of free spaces to later run algorithm to find optimal free positions and
            # append to parkingPositionList
            freeParkingPostiions.append(countpixelsList[i])
        else:
            parkingPositionList.append(countpixelsList[i])
    dp = [[-1,None]]*len(freeParkingPostiions)
    # run dynamic programming algorithm to choose the rest of parking positions into posList
    numOfFreeSpaces, freeSpacePositions = optimalFreePositions(0, freeParkingPostiions, [], dp)
    for pos in freeSpacePositions:
        parkingPositionList.append(pos)

    totalSpaces = len(parkingPositionList)

    # mark frame
    for pos in parkingPositionList:
        x,y = pos[0], pos[1]

        imgCrop = imgPro[pos[1]:pos[1]+pos[3], pos[0]:pos[0]+pos[2]]
        # cv2.imshow(str(x*y), imgCrop)
        count = cv2.countNonZero(imgCrop)
        cvzone.putTextRect(frame, str(count), (pos[0]+(pos[2]//2),pos[1]+(pos[3]//2)), scale = 1, thickness = 1, offset = 0)

        # if less than 250 free -> color green else color red
        if count < pixel_min:
            color = (0,255,0)
            thickness = 4
            freeSpace += 1
        else:
            color = (0,0,255)
            thickness = 2

        cv2.rectangle(frame, (pos[0], pos[1]), (pos[0] + pos[2], pos[1] + pos[3]), color, thickness)

    # cvzone.putTextRect(frame,  f'Free: {freeSpace}', (1500, 100), scale=4, thickness=3, offset=0, colorR=(255,0,0))
    free_spaces = freeSpace

def getFreeSpaces():
    global freeSpaces
    return freeSpaces

def getTotalSpaces():
    global totalSpaces
    return totalSpaces


# brighten the img. Was taken from:
# https://stackoverflow.com/questions/44752240/how-to-remove-shadow-from-scanned-images-using-opencv
def no_shadows(img):
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


def proccess_frame(frame, poslist):

    no_shadow = no_shadows(frame)

    # process img for testing in parking model
    imgGray = cv2.cvtColor(no_shadow, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (25, 25), 1)
    kernel = np.ones((3, 3), np.uint8)
    imgDilate = cv2.dilate(imgBlur, kernel, iterations=1)
    imgThreshold = cv2.adaptiveThreshold(imgDilate, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 16)

    cv2.imshow("imgThreshold", imgThreshold)


    checkParkingSpace(imgThreshold, frame, poslist)


def stream():
    global cap, outputFrame, lock
    # imgDilate = None

    while True:
        # read posList from parkingPositions with lock
        with lock_posList:
            try:
                with open('parkingPositions', 'rb') as f:
                    poslist = pickle.load(f)
            except:
                poslist = []

        if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
            cap.set(cv2.CAP_PROP_POS_FRAMES,0)

        success, frame = cap.read()
        if success:
            proccess_frame(frame, poslist)
            # frame = cv2.resize(frame, (1500, 850))
            with lock:
                outputFrame = frame.copy()


        cv2.imshow("Video", frame)
        # cv2.imshow("imgThreshold", imgDilate)

        k = cv2.waitKey(1)
        if k == ord('q'):
            exit()

def generate():
    global outputFrame, lock

    # loop over frames from the output stream
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue

            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            if not flag:
                continue

        # yield the output frame in the byte format
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')

