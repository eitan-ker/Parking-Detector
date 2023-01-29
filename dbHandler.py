import pymongo
from pymongo import MongoClient


class DbHandler:
    def __init__(self):
        cluster = MongoClient('mongodb://localhost:27017')
        self.__db = cluster['Parking_Finder']
        self.__collection_parking_areas = self.__db['parking_areas']
        self.__collection_parking_positions = self.__db['parking_positions']
        self.__parkingAreas = []
        self.__parkingPositions = []
        self.__initParkingAreas()
        self.__initParkingPositions()


    def __initParkingAreas(self):
        parkings_Areas = self.__collection_parking_areas.find({})
        for i in parkings_Areas:
            a, b, c, d = i['top'], i['right'], i['bottom'], i['left']
            self.__parkingAreas.append([a, b, c, d])

    def __initParkingPositions(self):
        parkings_Positions = self.__collection_parking_positions.find({})
        for i in parkings_Positions:
            a, b, c, d = i['left_up'], i['right_up'], i['right_down'], i['left_down']
            self.__parkingPositions.append([a, b, c, d])

    def addParkingArea(self, a, b, c, d):
        self.__collection_parking_areas.insert_one({"top": a, "right": b, "bottom": c, "left": d})

    def addParkingPosition(self, a, b, c, d):
        self.__collection_parking_positions.insert_one({"left_up": a, "right_up": b, "right_down": c, "left_down": d})

    def getParkingArea(self):
        return self.__parkingAreas

    def getParkingPositions(self):
        return self.__parkingPositions

    def deleteParkingArea(self, a, b, c, d):
        self.__collection_parking_areas.delete_one({"top": a, "right": b, "bottom": c, "left": d})

    def deleteParkingPosition(self, a, b, c, d):
        self.__collection_parking_positions.delete_one({"left_up": a, "right_up": b, "right_down": c, "left_down": d})


