# BioHack2024
**Cost-Efficient AI Prosthetic - Team 18**

Biohack Prosthetic Code

Overview: This project is designed to enhance prosthetic hand functionality by detecting objects using a camera and adjusting grip strength accordingly. The system uses YOLOv11 for object detection, processes the input through Python scripts, and communicates with a microcontroller via Bluetooth to control grip strength.

##Features:

Object Detection: Identifies objects in real time using YOLOv11.
Grip Strength Adjustment: Determines the necessary grip strength based on the detected object.
Arduino Compatibility for 3rd Party Cameras: Sends grip strength data to the prosthetic hand controller.

##Requirements:

`pip install -r requirements.txt`

##Training:

###Config:

in yolo/data/data.yml

`train: "your path to train/image"
val: "your path to val/image"
test: "your path to test/image"`

Change 

`model.train(data="yolo/data/data.yaml", epochs=100, imgsz=640, device="mps")`

to what fits your computer in this [link](https://docs.ultralytics.com/modes/train/#usage-examples)

Then 

`python train.py`


##Usage:

###Config:

Change this config values to your arduino connection and the model path

`config = {
        'arduino_port': '/dev/cu.usbmodem101',
        'servo_pins': {'thumb': 13, 'index': 11, 'middle': 9},
        'model_path': '/Users/hwangjunho/runs/detect/train10'
    }`

`python detect.py`

This will open the camera feed and begin real-time object detection.

The detected object near the prosthetic will be classified, and an appropriate grip strength value will be calculated.

Value goes through Arduino to adjust Servo motors on prosthetic

The program outputs logs for detected objects, grip strength values, and transmission status.

##Future Improvements

Implement more object categories for enhanced accuracy.

Optimize grip strength calculations based on real-world testing.

Improve latency for faster real-time processing.
