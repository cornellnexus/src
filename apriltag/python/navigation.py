# -----------------------------------------------------------------------------
# Team: Nexus at Cornell
# Author: Alan Hsiao
# Date: 12_20_2021
# Script: navigation.py
# Description: AprilTag detection with motor support
# -----------------------------------------------------------------------------

# Import libraries
from __future__ import division
from __future__ import print_function
from argparse import ArgumentParser
import cv2
import apriltag
import RPi.GPIO as GPIO
import time


# set up PWM pins and frequency
PWMR = 12
PWML = 13
freq = 50

# Set up PWM with frequency
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMR, GPIO.OUT)
GPIO.setup(PWML, GPIO.OUT)
pR = GPIO.PWM(PWMR, freq)
pL = GPIO.PWM(PWML, freq)


# Spin clockwise
def spin180_cw():
    pL.ChangeDutyCycle(6.5)
    pR.ChangeDutyCycle(6.5)
    time.sleep(4)


# Spin counter-clockwise
def spin180_ccw():
    pL.ChangeDutyCycle(8.5)
    pR.ChangeDutyCycle(8.5)
    time.sleep(4)


# Start PWM with 0 duty cycle
pR.start(0)
pL.start(0)


def main():
    # Define two parsers as in cam_calibrate_nexus.py
    parser = ArgumentParser(description="test apriltag Python bindings")
    parser2 = ArgumentParser(description="test apriltag Python bindings")

    parser.add_argument(
        "device_or_movie",
        metavar="INPUT",
        nargs="?",
        default=0,
        help="Movie to load or integer ID of camera device",
    )

    parser2.add_argument(
        "device_or_movie",
        metavar="INPUT",
        nargs="?",
        default=1,
        help="Movie to load or integer ID of camera device",
    )

    apriltag.add_arguments(parser)

    apriltag.add_arguments(parser2)

    options = parser.parse_args()
    options2 = parser2.parse_args()

    # Define two video feeds as in cam_capture_nexus.py
    try:
        cap = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
    except ValueError:
        cap = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)

    window = "Camera"
    cv2.namedWindow(window)

    window2 = "Camera2"
    cv2.namedWindow(window2)

    # Define two detectors
    detector = apriltag.Detector(options, searchpath=apriltag._get_demo_searchpath())
    detector2 = apriltag.Detector(options2, searchpath=apriltag._get_demo_searchpath())

    # instantiate algorithm
    dc = 6.5
    state = 0
    start_time = 0

    while True:
        # Read every frame
        success, frame = cap.read()
        success2, frame2 = cap2.read()

        if not success:
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)

        # Pass into detector
        detections, dimg = detector.detect(gray, return_image=True)
        detections2, dimg2 = detector2.detect(gray2, return_image=True)

        # Return length of detection
        num_detections = len(detections)
        num_detections2 = len(detections2)

        # state definition
        #
        # state 0 is instantiate start time
        # state 1 is finding
        # state 2 is rest from finding
        # state 3 is found

        if num_detections == 0 and num_detections2 == 0:
            if state == 0 or state == 3:
                start_time = time.time()
                state = 1
            elif state == 1 and time.time() > start_time + 4:
                state = 2
                start_time = time.time()
            elif state == 2 and time.time() > start_time + 2:
                state = 0

            if state == 1:
                pL.ChangeDutyCycle(7.2)
                pR.ChangeDutyCycle(7.2)
            else:
                pL.ChangeDutyCycle(0)
                pR.ChangeDutyCycle(0)
        else:
            state = 3
            pL.ChangeDutyCycle(0)
            pR.ChangeDutyCycle(0)

        # print(int(time.time() - start_time))

        # print('Detected {} tags with camera 1.\n'.format(num_detections))
        # print('Detected {} tags with camera 2.\n'.format(num_detections2))

        for i, detection in enumerate(detections):
            print("Detection {} of {}:".format(i + 1, num_detections))
            print()
            print(detection.tostring(indent=2))
            print()

        for i, detection2 in enumerate(detections2):
            print("Detection2 {} of {}:".format(i + 1, num_detections2))
            print()
            print(detection2.tostring(indent=2))
            print()

        overlay = frame // 2 + dimg[:, :, None] // 2
        overlay2 = frame2 // 2 + dimg2[:, :, None] // 2

        cv2.imshow(window, overlay)
        cv2.imshow(window2, overlay2)

        k = cv2.waitKey(1)

        if k == 27:
            break


if __name__ == "__main__":
    main()
