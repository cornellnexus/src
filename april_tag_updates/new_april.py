import imutils
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
left_right_dist = 25.5
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(1,cv2.CAP_DSHOW)
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_APRILTAG_36h11"])
arucoParams = cv2.aruco.DetectorParameters_create()
CACHED_PTS = None
CACHED_IDS = None
Line_Pts = None
measure = None
CACHED_PTS2 = None
CACHED_IDS2 = None
Line_Pts2 = None
measure2 = None
window = 'Camera'
cv2.namedWindow(window)
window2='Camera2'
cv2.namedWindow(window2)
while True:
    Dist = []
    ret, image = cap.read()
    image = imutils.resize(image, width=800)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image, arucoDict, parameters=arucoParams)
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
        for (markerCorner, markerId) in zip(corners, ids):
            print("[INFO] Marker detected1")
            corners_abcd = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
            topRightPoint = (int(topRight[0]), int(topRight[1]))
            topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
            bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))
            cv2.line(image, topLeftPoint, topRightPoint, (0, 255, 0), 2)
            cv2.line(image, topRightPoint, bottomRightPoint, (0, 255, 0), 2)
            cv2.line(image, bottomRightPoint, bottomLeftPoint, (0, 255, 0), 2)
            cv2.line(image, bottomLeftPoint, topLeftPoint, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0])//2)
            cY = int((topLeft[1] + bottomRight[1])//2)
            measure = abs(5/(topLeft[0]-cX))
            cv2.circle(image, (cX, cY), 4, (255, 0, 0), -1)
            cv2.putText(image, str(
                int(markerId)), (int(topLeft[0]-10), int(topLeft[1]-10)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
            Dist.append((cX, cY))
            # print(arucoDict)
            if len(Dist) == 0:
                if Line_Pts is not None:
                    Dist = Line_Pts
            if len(Dist) == 2:
                Line_Pts = Dist
            if len(Dist) == 2:
                cv2.line(image, Dist[0], Dist[1], (255, 0, 255), 2)
                ed = ((Dist[0][0] - Dist[1][0])**2 +
                ((Dist[0][1] - Dist[1][1])**2))**(0.5)
                cv2.putText(image, str(int(measure*(ed))) + "cm", (int(300), int(
                300)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
    cv2.imshow(window, image)

    Dist2 = []
    ret2, image2 = cap2.read()
    image2 = imutils.resize(image2, width=800)
    (corners2, ids2, rejected2) = cv2.aruco.detectMarkers(
        image2, arucoDict, parameters=arucoParams)
    if len(corners2) <= 0:
        if CACHED_PTS2 is not None:
            corners2 = CACHED_PTS2
    if len(corners2) > 0:
        CACHED_PTS2 = corners2
        if ids2 is not None:
           ids2 = ids2.flatten()
           CACHED_IDS2 = ids2
        else:
           if CACHED_IDS2 is not None:
              ids2 = CACHED_IDS2
        if len(corners2) < 2:
           if len(CACHED_PTS2) >= 2:
              corners2 = CACHED_PTS2
        for (markerCorner, markerId) in zip(corners2, ids2):
            print("[INFO] Marker detected2")
            corners_abcd = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
            topRightPoint = (int(topRight[0]), int(topRight[1]))
            topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
            bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))
            cv2.line(image2, topLeftPoint, topRightPoint, (0, 255, 0), 2)
            cv2.line(image2, topRightPoint, bottomRightPoint, (0, 255, 0), 2)
            cv2.line(image2, bottomRightPoint, bottomLeftPoint, (0, 255, 0), 2)
            cv2.line(image2, bottomLeftPoint, topLeftPoint, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0])//2)
            cY = int((topLeft[1] + bottomRight[1])//2)
            measure2 = abs(5/(topLeft[0]-cX))
            cv2.circle(image2, (cX, cY), 4, (255, 0, 0), -1)
            cv2.putText(image2, str(
                int(markerId)), (int(topLeft[0]-10), int(topLeft[1]-10)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
            Dist.append((cX, cY))
            # print(arucoDict)
            if len(Dist2) == 0:
                if Line_Pts2 is not None:
                    Dist2 = Line_Pts2
            if len(Dist2) == 2:
                Line_Pts2 = Dist2
            if len(Dist2) == 2:
                cv2.line(image2, Dist2[0], Dist2[1], (255, 0, 255), 2)
                ed = ((Dist2[0][0] - Dist2[1][0])**2 +
                ((Dist2[0][1] - Dist2[1][1])**2))**(0.5)
                cv2.putText(image2, str(int(measure2*(ed))) + "cm", (int(300), int(
                300)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
    cv2.imshow(window2, image2)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
cv2.destroyAllWindows()
cap.stop()