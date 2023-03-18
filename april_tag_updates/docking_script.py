expected_dist = 37
known_width = 4.18 ##inches
focal_length = 1.37478492e+03
from electrical.motor_controller import BasicMotorController
import numpy as np
import cv2
import time 
from enum import Enum
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_APRILTAG_36h11"])
arucoParams = cv2.aruco.DetectorParameters_create()
cap = cv2.VideoCapture(0)
window = 'Camera'
cv2.namedWindow(window)
def distance_to_camera(knownWidth, focalLength, perWidth):
  # compute and return the distance from the maker to the camera
  return (knownWidth * focalLength) / perWidth

def direction_of_tag(corner,image):
  (_, _, _, _) = corner
  _,w = image.shape[0], image.shape[1]
  cx, _ = int(960),int(540)
  if  bottomLeft[0] > cx:
    return "right"
  else:
    return "left"

idDict = {1: "R1", 2: "R2", 3: "R3"}#TODO add all the other tags
visited = []
timeout =  10
angle_zero = 10
class Steps_Done(Enum):
    one = False
    two = False
    three = False
    four = False
motor = BasicMotorController(True)
start_time = None
while True:
    if start_time is None:
        start_time = time.time()
    ret, image = cap.read()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    if len(corners) == 0:
      if time.time() - start_time > timeout and Steps_Done.one is False:
        #Step 1 - print("Check for april tags rn by rotating in circle for 15 seconds.")
          motor.turn_right()
      if Steps_Done.one and Steps_Done.two is False:
        #Step 2 - print(go back pls until len(corners) = 1)
        motor.reverse()
    if len(corners) > 0:
      if Steps_Done.one:
        Steps_Done.two = True
      start_time = time.time()
      for (corner, markerId) in zip(corners, ids):
          # print(direction_of_tag(corner[0],image))
          markerId = float(markerId)
          (topLeft, topRight, bottomRight, bottomLeft) = corner[0]
          pixel_width = np.sqrt((bottomRight[0] - bottomLeft[0])**2 + (bottomRight[1] - bottomLeft[1])**2)
          depth = distance_to_camera(known_width,focal_length,pixel_width)
          # print(f"{depth_distance} in")
          cxt,cyt = int((topLeft[0] + bottomRight[0])//2),int((topLeft[1] + bottomRight[1])//2)
          cx, cy = int(960),int(540)
          cv2.circle(image, (cxt, cyt), radius = 20, color = (0, 0, 255), thickness = -1)
          cv2.circle(image, (cx, cy), radius = 20, color = (0, 0, 255), thickness = -1)
          cv2.line(image,(cxt, cyt),(cx, cy), color = (0, 0, 0), thickness = 2)
          ratio= known_width/pixel_width
          dpix = np.sqrt((cx-cxt)**2+(cy-cyt)**2)
          horiz_dist = dpix * ratio
          midpoint = (int((cx + cxt) //2 ), int((cy + cyt) //2 - 25 ))
          midpoint2 = (int((cx + cxt) //2), int((cy + cyt) //2 + 50 ))
          midpoint3 = (int((cx + cxt) //2), int((cy + cyt) //2 - 100 ))
          cv2.putText(image, f"{horiz_dist: .2f} in", midpoint, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 101, 255), 2)
          angle = np.degrees(np.arcsin(horiz_dist/(depth)))
          cv2.putText(image, f"{angle:.2f} degrees", midpoint2, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 101, 255), 2)
          cv2.putText(image, f"{depth:.2f} in (depth)", midpoint3, cv2.FONT_HERSHEY_COMPLEX, 1, (255,0, 255), 2)
          #Step 1 starts here
          if idDict.get(markerId) == "R2": ##TODO: Add B2, L2, F2 in the future
            #Step 1 is over. Start Step 2: move forward until tag out of sight. "
            if angle <= angle_zero:
              motor.go_forward()
              Steps_Done.one = True
            else:
              # Step 1 - "Keep rotating till we reach center of middle tag"
              if direction_of_tag(corner[0],image) == "right":
                motor.turn_left()
              else: 
                motor.turn_right()
          else:
            #All of this is Step 1 
            if idDict.get(markerId) not in visited: 
              if(angle <= angle_zero):
                visited.append(idDict.get(markerId))
              else:
                if direction_of_tag(corner[0],image) == "right":
                  motor.turn_left()
                else:
                  motor.turn_right()
          #Step 3 + TODO: test how much turn_right and turn_left turns with our current robot + assume turn_right/left is 10 degrees
          if Steps_Done.two: 
            num_calls = angle // 10
            if direction_of_tag(corner[0],image) == "right":
              for i in range(num_calls):
                motor.turn_right()
            else: 
              for i in range(num_calls):
                motor.turn_left()
            Steps_Done.three = True
          
          #step 4: reverse by 30 inches 
          if Steps_Done.three: 
            if (depth < 30):
              motor.reverse()
            Steps_Done.four = True

    if Steps_Done.four == True:
      break
             
    cv2.imshow("Camera", image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
      break
cap.release()
cv2.destroyAllWindows()
  


  

