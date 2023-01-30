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

# Create a Flask instance and enable CORS
app = Flask(__name__)
CORS(app)

# Define a route for the index page
@app.route("/")
def index():
    # Call the getTotalSpaces and getFreeSpaces methods from the parking_model module
    totalSpaces = model.getTotalSpaces()
    freeSpaces = model.getFreeSpaces()
    # Render the HTML template with the total spaces and free spaces values
    return render_template('/HTML/index.html',  total_spaces=totalSpaces, free_spaces=freeSpaces)

# Define a route for the video feed
@app.route("/video_feed")
def video_feed():
    # Return a response with the generated video
    return Response(model.generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# Define a route for the parking space information
@app.route("/info")
def info():
    # Call the getFreeSpaces and getTotalSpaces methods from the parking_model module
    freeSpaces = model.getFreeSpaces()
    totalSpaces = model.getTotalSpaces()
    # Return a JSON object with the total spaces and free spaces values
    return jsonify(totalSpaces_value=totalSpaces, freeSpaces_value=freeSpaces)

# Main function
if __name__ == "__main__":

    # Define the stream video source
    stream = 'resultvideo2013.avi'
    # stream = 'http://eitancamhome:eitancamhome@10.100.102.10:6677/video'

    # Create an instance of the DbHandler
    db = dbHandler.DbHandler()

    # Define the weights for the parking space detector
    weights = 'yolov5s'

    # Create an instance of the parking_model class
    model = parking_model.Model(stream, db, weights)
    # Start a new thread for the stream method
    t1 = threading.Thread(target=model.stream)
    t1.daemon = True
    t1.start()

    # Create an instance of the parkingPositionsDetector class
    detector = parkingPositionsDetector.Detector(stream, db)
    # Start a new thread for the detectionAlgorithm method
    t2 = threading.Thread(target=detector.detectionAlgorithm)
    t2.daemon = True
    t2.start()

    # Start the Flask app
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True, use_reloader=False)
