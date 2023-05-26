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
# image = cv2.imread("april_test.jpg")
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth
while True:
 		ret, image = cap.read()
 		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
 		if corners != ():
 			(topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
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
 			for corner in corners:
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
 		cv2.imshow("yolo", image)
 		key = cv2.waitKey(1) & 0xFF
 		if key == ord('q'):
			break
cap.close()
cv2.destroyAllWindows()