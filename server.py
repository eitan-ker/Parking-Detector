from flask import Flask, render_template, request, Response, jsonify
import parking_model
import parkingPositionsDetector
import dbHandler
import threading
from flask_cors import CORS
import pymongo
from pymongo import MongoClient

import cProfile
import time



app = Flask(__name__)
CORS(app)


@app.route("/")
def index():
    totalSpaces = model.getTotalSpaces()
    freeSpaces = model.getFreeSpaces()
    return render_template('/HTML/index.html',  total_spaces=totalSpaces, free_spaces=freeSpaces)

@app.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(model.generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/info")
def info():
    freeSpaces = model.getFreeSpaces()
    totalSpaces = model.getTotalSpaces()
    return jsonify(totalSpaces_value=totalSpaces, freeSpaces_value=freeSpaces)



if __name__ == "__main__":

    stream = 'resultvideo2011DONE.avi'
    # stream = 'http://eitancamhome:eitancamhome@10.100.102.10:6677/video'

    db = dbHandler.DbHandler()

    weights = 'yolov5s'

    model = parking_model.Model(stream, db, weights)
    t1 = threading.Thread(target=model.stream)
    t1.daemon = True
    t1.start()

    detector = parkingPositionsDetector.Detector(stream, db)
    t2 = threading.Thread(target=detector.detectionAlgorithm)
    t2.daemon = True
    t2.start()

    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True, use_reloader=False)


