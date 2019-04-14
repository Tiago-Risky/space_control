import time
import datetime
import serial
from influxdb import InfluxDBClient
import random

import cv2
from imutils.video import VideoStream
import imutils
import numpy as np

#Arduino device
arduinoDevice = "/dev/ttyUSB0"
#arduinoDevice = "/dev/ttyACP0"

#debug
debug = False

#Period in seconds for every loop
period = 120

#thresholds
threshold_radar = 10 * 60
threshold_detection = 10 * 60

#flags
flag_radar = False
flag_radar_when = 0 #epoch time
flag_detection = False
flag_detection_when = 0 #epoch time

#time variables
last_arduino_read = 0 #epoch time
last_camera_read = 0 #epoch time at start of reading

#
s = "0"
r = False
h = "0.0"
t = "0.0"
l = "0"


conn = None

def loop():
    global flag_radar
    global s
    global r
    global h
    global t
    global l
    #main loop
    last_arduino_read = time.time()

    d = 0
    #s,r,h,t,l = readArduino()
    print(s,r,h,t,l)

    if(r):
        flag_radar = True

    if(flag_radar):
        d = cameraLoop()

    writeToDB(s,r,h,t,l,d)

    now = time.time()
    if now < last_arduino_read+period:
        now = time.time()
        loop_time = now-last_arduino_read 
        while loop_time > 0:
            s,r,h,t,l = readArduino()
            if r:
                flag_radar = True
                break

def cameraLoop():
    print("Processing image")
    if not debug:
        people = Camera().main()
    else:
        people = random.randint(0,5)
    conn.flushInput()
    conn.flushOutput()
    print(people)
    return people

def bootArduino():
    global conn
    try:
        conn = serial.Serial(arduinoDevice, 9600)
    except serial.SerialException as e:
        print("Fail to connect: {}".format(e))
        exit()
    time.sleep(5)

def readArduino():
    global conn
    dataarr = []
    while(len(dataarr)!=5):
        try:
            data = conn.readline().decode()
        except serial.SerialException as e:
            print("Failed to read, trying again. {}".format(e))
            data = ""
        print(data)
        data = data.strip()
        dataarr = data.split(",")
        print(dataarr)
        if len(dataarr) == 5:
            s,r,h,t,l = dataarr

    if r=="1":
        r_bool = True
    else:
        r_bool = False
    #Returns temperature, humidity, luminosity, movement
    return s,r_bool,h,t,l

#Takes temperature, humidity, luminosity, movement, people
def writeToDB(s,r,h,t,l,d):
    dt = datetime.datetime.now()
    dt_str = dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    if r:
        r_val = "true"
    else:
        r_val = "false"

    client = InfluxDBClient(host='192.168.100.63',port=8086)
    client.switch_database('spacecontrol')
    
    json_body = [
    {
    "measurement": "radar",
    "tags": {"sensor":s},
    "time": dt_str,
    "fields": {
        "value": r_val
        }
    },
    {
    "measurement": "humidity",
    "tags": {"sensor":s},
    "time": dt_str,
    "fields": {
        "value": h
        }
    },
    {
    "measurement": "temperature",
    "tags": {"sensor":s},
    "time": dt_str,
    "fields": {
        "value": t
        }
    },
    {
    "measurement": "lumens",
    "tags": {"sensor":s},
    "time": dt_str,
    "fields": {
        "value": l
        }
    },
    {
    "measurement": "detection",
    "tags": {"sensor":"1"},
    "time": dt_str,
    "fields": {
        "value": str(d)
        }
    }
    ]

    client.write_points(json_body)


class Camera():
    colours = None
    classes = None
    classes_trim = None

    #config
    yolov3_classes = "yolov3.txt"
    yolov3_classes_trimmed = "yolov3-trimmed.txt"
    yolov3_config = "yolov3.cfg"
    yolov3_weights = "yolov3.weights"

    def main(self):
        vs = VideoStream(usePiCamera=True).start()

        with open(self.yolov3_classes, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        with open(self.yolov3_classes_trimmed, 'r') as f:
            self.classes_trimmed = [line.strip() for line in f.readlines()]

        self.colours = np.random.uniform(0, 255, size=(len(self.classes), 3))

        net = cv2.dnn.readNet(self.yolov3_weights, self.yolov3_config)

        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()
        vs.stop()
        frame = imutils.resize(frame, width=640,inter=cv2.INTER_CUBIC)

        # grab the frame dimensions and convert it to a blob
        Height, Width = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 0.00392, (640,480), (0,0,0), True, crop=False)
        # pass the blob through the network and obtain the detections and
        # predictions
        net.setInput(blob)

        outs = net.forward(self.get_output_layers(net))

        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if self.classes[class_id] in self.classes_trimmed and confidence > 0.5: #need to further develop from here
                    center_x = int(detection[0] * Width)# we can use these centers to know where the person is
                    center_y = int(detection[1] * Height)# then we can improve the detection output to send the positional data
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

        for i in indices:
            i = i[0]
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
            self.draw_prediction(frame, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))

        #We need to send this
        people = len(indices) 

        return people

    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def draw_prediction(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):
        label = str(self.classes[class_id])
        colour = self.colours[class_id]
        cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), colour, 2)
        cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

bootArduino()
while(True):
    loop()