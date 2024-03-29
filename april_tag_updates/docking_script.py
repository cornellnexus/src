known_width = 4.18  ##inches
focal_length = 1.37478492e03  ##pixels
from electrical.motor_controller import BasicMotorController
import numpy as np
import cv2
import time
from enum import Enum


class Steps(Enum):
    """
    An enumeration of the different steps in the April Tag Docking Algorithm

    Arguments:
        cx,cy: center point of an image captured by a camera with 1080p resolution
        idDict: dictionary of ids of April Tags mapped to corresponding string values.
        visited: set of non-centered april tags that have been already been seen and aligned with camera.
        step: the current step the docking algorithm is on
        motor: motor controller with commands to physically move the robot
        start_time: the timestamp when script execution began
        angle: current angle between center of camera and center of april tag
        depth: distance between center of camera and center of april tag
        timeout: experimentally determined 10-second duration after which, if the AprilTag is not detected, the rotation process begins
        angle_min: the minimum, physically possible angle between center of camera and center of april tag
        depth_min: the minimum, physically possible distance between center of camera and center of april tag
        arucoDict: A predefined ArUco dictionary containing the AprilTag 36h11 family.
        arucoParams: parameters that define various settings and thresholds used during the marker detection process.
    """

    Step1 = 1
    Step2 = 2
    Step3 = 3
    Step4 = 4
    Done = 5


cx, cy = int(960), int(540)
idDict = {1: "R1", 2: "R2", 3: "R3"}  # TODO add all the other tags
visited = set()
step = Steps.Step1
motor = BasicMotorController(True)  # TODO replace with MotorController instance
start_time = None
angle = float("inf")
depth = float("inf")
timeout = 10  ##seconds
angle_min = 10  ##degrees
depth_min = 8  ##inches
aligned_with_tag = lambda angle: angle <= angle_min
close_to_tag = lambda depth: depth <= depth_min
aligned = False
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
arucoParams = cv2.aruco.DetectorParameters_create()
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
window = "Camera"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 960, 540)


class Direction(Enum):
    """
    A class representing the two possible movements the camera can turn, right and left.
    """

    RIGHT = 1
    LEFT = 2


def distance_to_camera(known_width, focal_length, pixel_width):
    """
    Returns the perpendicular distance from the center of camera to the center of an April tag.
    Arguments:
      known_width: physically measured width of a 36h11 april tag
      focal_length: focal length of camera found using camera_calibration.py
      pixel_width: width of 36h11 april tag expressed in pixels
    """
    return (known_width * focal_length) / pixel_width


def direction_of_tag(corner):
    """
    Returns whether tag is right or left of center of image.
    Arguments:
      corner: corner of tag
    """
    (_, _, _, bottomLeft) = corner
    if bottomLeft[0] > cx:
        return Direction.RIGHT
    else:
        return Direction.LEFT


def visualization(corner, markerId):
    """
    Returns image of tag(s) with labelled distance and angle from center of 1080p resolution camera to tag.
    Arguments:
      corner: corner of tag
      markerId: id of tag
    """
    markerId = float(markerId)
    (topLeft, _, bottomRight, bottomLeft) = corner[0]
    pixel_width = np.sqrt(
        (bottomRight[0] - bottomLeft[0]) ** 2 + (bottomRight[1] - bottomLeft[1]) ** 2
    )
    depth = distance_to_camera(known_width, focal_length, pixel_width)
    cxt, cyt = int((topLeft[0] + bottomRight[0]) // 2), int(
        (topLeft[1] + bottomRight[1]) // 2
    )
    cv2.circle(image, (cxt, cyt), radius=20, color=(0, 0, 255), thickness=-1)
    cv2.circle(image, (cx, cy), radius=20, color=(0, 0, 255), thickness=-1)
    # Draw a line between the center of the tag and the center of the camera
    cv2.line(image, (cxt, cyt), (cx, cy), color=(0, 0, 0), thickness=2)
    ratio = known_width / pixel_width
    dpix = np.sqrt((cx - cxt) ** 2 + (cy - cyt) ** 2)
    horiz_dist = dpix * ratio
    # Define midpoints for text placement of horizontal distance., angle, and depth distance to tag
    midpoint = (int((cx + cxt) // 2), int((cy + cyt) // 2 - 25))
    midpoint2 = (int((cx + cxt) // 2), int((cy + cyt) // 2 + 50))
    midpoint3 = (int((cx + cxt) // 2), int((cy + cyt) // 2 - 100))
    cv2.putText(
        image,
        f"{horiz_dist: .2f} in",
        midpoint,
        cv2.FONT_HERSHEY_COMPLEX,
        1,
        (0, 101, 255),
        2,
    )
    angle = np.degrees(np.arcsin(horiz_dist / (depth)))
    cv2.putText(
        image,
        f"{angle:.2f} degrees",
        midpoint2,
        cv2.FONT_HERSHEY_COMPLEX,
        1,
        (0, 101, 255),
        2,
    )
    cv2.putText(
        image,
        f"{depth:.2f} in (depth)",
        midpoint3,
        cv2.FONT_HERSHEY_COMPLEX,
        1,
        (255, 0, 255),
        2,
    )
    return markerId, angle, depth


while True:
    if start_time is None:
        start_time = time.time()
    ret, image = cap.read()
    cx, cy = image.shape[1] // 2, image.shape[0] // 2
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image, arucoDict, parameters=arucoParams
    )  # returns corner coordinates of detected markers (corners), their corresponding IDs (ids), and the coordinates of rejected marker candidates (rejected).
    is_april_tag_in_view = len(corners) > 0
    if not is_april_tag_in_view:
        # After we go forward until tag's no longer seen in Step 1, if we are aligned and close to the tag, Step 1 terminates and we proceed to Step 2.
        if aligned_with_tag(angle) and close_to_tag(depth):
            step == Steps.Step2
        # Step 1 - If no april tags have been spotted for 15 seconds (timeout), we rotate until we spot one.
        if time.time() - start_time > timeout and step == Steps.Step1:
            motor.turn_right()  ##TODO ask electrical to make time a parameter of this function. Then, we can rotate for a specified time limit and return to return phase if no tags spotted.
        # Step 2 - go back until we see at least 1 april tag. After motor reverses, Step 2 terminates.
        if step == Steps.Step2:
            motor.reverse()
    else:
        # Step 2 terminates by reversing until the center tag is seen. Proceed to Step 3
        if step == Steps.Step2:
            step = Steps.Step3
        start_time = time.time()
        for corner, markerId in zip(corners, ids):
            markerId, angle, depth = visualization(corner, markerId)
            seen_center_tag = idDict.get(markerId) == "R2"
            # Step 1 starts here.
            if step == Steps.Step1:
                aligned = aligned_with_tag(angle)
                if seen_center_tag and aligned:
                    # Once the camera's aligned with the center tag, the robot goes forward until the tag is no longer visible.
                    motor.go_forward()
                else:  # If we aren't aligned with the center tag, we are still in Step 1. We align with the current tag (whether it's the center or not)
                    if idDict.get(markerId) not in visited:
                        if aligned:
                            visited.add(idDict.get(markerId))
                        else:
                            if direction_of_tag(corner[0]) == Direction.RIGHT:
                                motor.turn_left()
                            else:
                                motor.turn_right()
            # Step 3 turns by 'angle' degrees to make a straight line. We assume 1 call of turn_right/left is 3 degrees. TODO: test how much turn_right and turn_left turns with our current robot
            if step == Steps.Step3:
                num_calls = max(int(angle // 3), 1)
                if direction_of_tag(corner[0]) == Direction.RIGHT:
                    for i in range(num_calls):
                        motor.turn_right()
                else:
                    for i in range(num_calls):
                        motor.turn_left()
                step == Steps.Step4

            # step 4: reverse by some experimentally determined constant -  15 inches
            if step == Steps.Step4:
                if depth <= 15:
                    motor.reverse()
                else:
                    step == Steps.Done

    if step == Steps.Done:
        break

    cv2.imshow("Camera", image)
    key = cv2.waitKey(1) & 0xFF
    # user terminates script TODO: end of docking phase should terminate script
    if key == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
