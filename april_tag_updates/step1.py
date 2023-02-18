expected_dist = 35+2
known_width = 4.18 ##inches
focal_length = 1.37478492e+03
import imutils
import numpy as np
import cv2
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
# image = cv2.imread("april_test.jpg")
def distance_to_camera(knownWidth, focalLength, perWidth):
  # compute and return the distance from the maker to the camera
  return (knownWidth * focalLength) / perWidth
def angle():
  CACHED_PTS = None
  CACHED_IDS = None
  Line_Pts = None
  measure = None
  idDict = {1: "F1", 0: "F2", 3: "F3"}
  visited = []
  done = False
  while True:
      ret, image = cap.read()
      (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
      if len(corners) <= 0:
        if CACHED_PTS is not None:
          corners = CACHED_PTS
      if len(corners) > 0:
        CACHED_PTS = corners
        if ids is not None:
          ids = ids.flatten()
          CACHED_IDS = ids
        else:
          if CACHED_IDS is not None:
            ids = CACHED_IDS
        if len(corners) < 2:
          if len(CACHED_PTS) >= 2:
            corners = CACHED_PTS
        for (corner, markerId) in zip(corners, ids):
            (topLeft, topRight, bottomRight, bottomLeft) = corner[0]
            pixel_width = np.sqrt((bottomRight[0] - bottomLeft[0])**2 + (bottomRight[1] - bottomLeft[1])**2)
            vert_distance = distance_to_camera(known_width,focal_length,pixel_width)
            # print(f"{vert_distance} in")
            cxt,cyt = int((topLeft[0] + bottomRight[0])//2),int((topLeft[1] + bottomRight[1])//2)
            cx, cy = int(960),int(540)
            cv2.circle(image, (cxt, cyt), radius = 20, color = (0, 0, 255), thickness = -1)
            cv2.circle(image, (cx, cy), radius = 20, color = (0, 0, 255), thickness = -1)
            cv2.line(image,(cxt, cyt),(cx, cy), color = (0, 0, 0), thickness = 2)
            # print(cxt, cyt)
            ratio= known_width/pixel_width
            # print(ratio)
            dpix = np.sqrt((cx-cxt)**2+(cy-cyt)**2)
            horiz_dist = dpix * ratio
            midpoint = (int((cx + cxt) //2 ), int((cy + cyt) //2 - 25 ))
            midpoint2 = (int((cx + cxt) //2), int((cy + cyt) //2 + 50 ))
            midpoint3 = (int((cx + cxt) //2), int((cy + cyt) //2 - 100 ))
            cv2.putText(image, f"{horiz_dist: .2f} in", midpoint, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 101, 255), 2)
            # print(d_in)
            angle = np.degrees(np.arcsin(horiz_dist/(vert_distance)))
            cv2.putText(image, f"{angle:.2f} degrees", midpoint2, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 101, 255), 2)
            cv2.putText(image, f"{vert_distance:.2f} in (vert)", midpoint3, cv2.FONT_HERSHEY_COMPLEX, 1, (255,0, 255), 2)
            # print(f"{angle} degrees")
            if idDict.get(markerId) == "F2": 
              if angle < 10:
                print("Step 1 is over")
                done = True
                break
              else:
                print("Keep rotating till we reach center of middle tag")
            else:
              if idDict.get(markerId) not in visited: 
                if idDict.get(markerId) is not None:
                  if(angle < 10):
                    print("aligned with the center of " + idDict.get(markerId))
                    visited.append(idDict.get(markerId))
                  else:
                    print("keep rotating to align with center of " + idDict.get(markerId))
                else:
                  print("wait 10 seconds. rotate in a circle")
              else:
                print("We have seen this tag before. Do not rotate.")
                print(visited)

      if done == True:
        break       
      cv2.imshow("yolo", image)
      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        break
  cap.release()
  cv2.destroyAllWindows()
  

angle()
  

