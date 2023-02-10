# Parking Detector




## Project Description

Parking Detector is a **computer vision** application built with **Python** that detects available parking spots and keeps track of occupied spots in real-time allowing drivers to quickly find available spots.

The application also provides a web interface built with **Flask**, which displays the parking spot availability information. The information is stored in a **MongoDB** database, making it easy to retrieve and manipulate. 

The application utilizes **threading and multiprocessing** to optimize performance, ensuring that the application runs smoothly even with a high volume of data. The application also establishes an HTTP connection to an IP camera for obtaining real-time video feeds, providing an up-to-date view of the parking lot.

## Key Features

1. Python
2. Computer Vision
3. Flask web app
4. MongoDB database
5. Threading and Multiprocessing
6. Dynamic Programming
7. HTTP connection to IP camera



## Classes

**All classes are designed following OOP principles like Encapsulation and Abstraction.**

### Server

This is a **python** script for a **Flask** web application that provides a parking lot monitoring system. The main functionality of this script is as follows:

It creates an instance of the parking_model class and starts a new thread for the stream method, which generates a video stream of the parking lot and updates the free and total spaces information.

It creates an instance of the parkingPositionsDetector class and starts a new thread for the detectionAlgorithm method, which detects the parking spaces in the video stream and updates the occupancy information in the database.

It creates an instance of the DbHandler class and uses it to connect to a database, which stores the parking space information.

It sets up four routes for the web application:

'/video_feed' returns the video feed generated by the model.generate() method.

'/info' returns a JSON object with the total and free spaces information.

'/' returns the video feed in the default route.


### Parking Model

This Class is an implementation of a parking lot monitoring system with the help of **YOLOv5** object detection model. The **YOLOv5** model is loaded using PyTorch Hub and the video feed is obtained from a given stream. A **database** class is used to retrieve the parking areas and positions from a database.

The class has several methods to process the video frames. One method extracts occupied positions from the video frame using the **YOLOv5** model, another method marks the occupied and free parking spaces in the video frame, another method calculates the number of occupied and free parking spaces in the parking lot.

The code also implements an **optimized version of the maximum independent set problem** to determine the maximum number of non-intersecting parking spaces in the parking lot for selecting free parking spaces. The solution is obtained using **dynamic programming**.

### Parking Position Detector

The "Detector" class is for detecting and tracking objects (presumably vehicles) in a video stream. It uses the **OpenCV** library to process the video stream and a background subtractor to **detect changes** in the video frame. The class also keeps track of parking areas and parking positions, which are stored in a database. Parking areas are defined by marking 4 points in the video stream, and parking positions are defined as vehicles that are detected within the parking areas. 

The class has functions for marking and deleting parking areas, as well as for adding and deleting parking positions. The class also has functions for drawing parking areas and potential parking positions on the video stream.

### Tracker

The Tracker class is used for tracking objects in a parking area and **minimizing the number of parking positions** needed. The class tracks the cars in the parking area, saves the positions of each car, and returns the max area size parking position when the car stops moving.

### DbHandler

This class is responsible for handling communication with a MongoDB database. It contains a number of methods for adding, retrieving and deleting parking areas and parking positions from the database. The class makes use of the PyMongo library to interact with the MongoDB database, which is running on the local machine.
