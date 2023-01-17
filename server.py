from flask import Flask, render_template, request, Response
import parking_model
import parkingPositionsDetector
import threading
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route("/")
def index():
    return render_template('/HTML/index.html')

@app.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(parking_model.generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/info")
def info():
    freeSpaces = parking_model.getFreeSpaces()
    totalSpaces = parking_model.getTotalSpaces()
    return render_template('/HTML/info.html', freeSpaces=freeSpaces, totalSpaces=totalSpaces)

if __name__ == "__main__":
    t1 = threading.Thread(target=parking_model.stream)
    t1.daemon = True
    t1.start()
    t2 = threading.Thread(target=parkingPositionsDetector.detectionAlgorithm)
    t2.daemon = True
    t2.start()
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True, use_reloader=False)
