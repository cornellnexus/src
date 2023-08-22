#-----------------------------------------------------------------------------
# Team: Nexus at Cornell
# Author: Alan Hsiao
# Date: 12_20_2021
# Script: Cam_calibrate_nexus.py
# Description: Two camera AprilTag detection with distance/angle approximation
#-----------------------------------------------------------------------------

#import libraries
from __future__ import division
from __future__ import print_function
from argparse import ArgumentParser
import cv2
import apriltag
import math

def main():
    # ArgumentParser is used to define command line interfaces in Python
    # Define two ArgumentParsers to pass in the AprilTag detection parameters                         
    parser = ArgumentParser(description='test apriltag Python bindings')
    parser2 = ArgumentParser(description='test apriltag Python bindings')
    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,help='Movie to load or integer ID of camera device')
    parser2.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=1,help='Movie to load or integer ID of camera device')

    #Link parser to the AprilTag libraries
    apriltag.add_arguments(parser)
    apriltag.add_arguments(parser2)
    options = parser.parse_args()
    options2 = parser2.parse_args()
    
    # OpenCV video capture module for passing video feed to the script 
    try:
        cap = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
    except ValueError:
        cap = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)

    # Instantiating 2 windows for video feeds to be displayed on desktop
    window = 'Camera'
    cv2.namedWindow(window)
    window2 = 'Camera2'
    cv2.namedWindow(window2)
    
    # Define the two AprilTag detectors by passing in parser options
    detector = apriltag.Detector(options, searchpath = apriltag._get_demo_searchpath())
    detector2 = apriltag.Detector(options2, searchpath = apriltag._get_demo_searchpath())

    # Main Method Loop
    while True:

	  # OpenCV Capture frame by frame
        success, frame = cap.read()
        success2, frame2 = cap2.read()

	  # If capture frame does not work, program exit
        if not success:
            Break
        if not success2:
            break

	  # Convert captured frames to gray to save processor power/detect faster
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_RGB2GRAY)

	  # Pass gray images to the AprilTag detector module
        detections, dimg = detector.detect(gray, return_image=True)
        detections2, dimg2 = detector2.detect(gray2, return_image=True)
        
	  # Returned array length from the detector is the number of detections
        num_detections = len(detections)
        num_detections2 = len(detections2)

  print('Detected {} tags with camera1.\n'.format(num_detections))
  print('Detected {} tags with camera2.\n'.format(num_detections2))
        
	  # For each AprilTag detection by Camera 1
        for i, detection in enumerate(detections):
            
	      # Print the TagID 
            print("[0] Tag ID: ", detection.tag_id)
		

# Calculate edge vector lengths by subtracting X coordinates from 
# each other and y coordinates from each other. For example, top 
# edge X length is calculated by taking the subtracting the top 
# left X-coordinate from the top right X-coordinate. 

            top_edge_x=detection.corners[0][0]-detection.corners[1][0]
            top_edge_y=detection.corners[0][1]-detection.corners[1][1]
            
            right_edge_x=detection.corners[1][0]-detection.corners[2][0]
            right_edge_y=detection.corners[1][1]-detection.corners[2][1]   
            
            bottom_edge_x=detection.corners[2][0]-detection.corners[3][0]
            bottom_edge_y=detection.corners[2][1]-detection.corners[3][1]
          
            left_edge_x=detection.corners[3][0]-detection.corners[0][0]
            left_edge_y=detection.corners[3][1]-detection.corners[0][1]

		# Calculate dot product of each corner of the AprilTag square
            dot_prod_TR = top_edge_x*right_edge_x+top_edge_y*right_edge_y
            dot_prod_RB = right_edge_x*bottom_edge_x+right_edge_y*bottom_edge_y
            dot_prod_BL = bottom_edge_x*left_edge_x+bottom_edge_y*left_edge_y
            dot_prod_LT = left_edge_x*top_edge_x+left_edge_y*top_edge_y
            
		# Define angle approximation to be the maximum dot product value
		# normalized by the top edge length multiplied by a scalar factor
            angle_approx = max(abs(dot_prod_TR), abs(dot_product_LT))/(2*((top_edge_x/2)**2))*50
            print("[0] Angle Approximation: ", angle_approx)
            
		# Define estimated distance as the top edge length multiplied by 
		# angle approximation scaling divided by a scalar factor
            estimated_distance = (135/(abs(top_edge_x))*10*math.cos(math.radians(angle_approx)))
            estimated_distance = '{0:.4g}'.format(estimated_distance)
            print("[0] Estimated Distance: ", estimated_distance)

	  # For each AprilTag detection by Camera 1
        for i, detection2 in enumerate(detections2):
            
	      # Print the TagID 
            print("[1] Tag ID: ", detection2.tag_id)
		

# Calculate edge vector lengths by subtracting X coordinates from 
# each other and y coordinates from each other. For example, top 
# edge X length is calculated by taking the subtracting the top 
# left X-coordinate from the top right X-coordinate. 

            top_edge_x=detection2.corners[0][0]-detection2.corners[1][0]
            top_edge_y=detection2.corners[0][1]-detection2.corners[1][1]
            
            right_edge_x=detection2.corners[1][0]-detection2.corners[2][0]
            right_edge_y=detection2.corners[1][1]-detection2.corners[2][1]   
            
            bottom_edge_x=detection2.corners[2][0]-detection2.corners[3][0]
            bottom_edge_y=detection2.corners[2][1]-detection2.corners[3][1]
          
            left_edge_x=detection2.corners[3][0]-detection2.corners[0][0]
            left_edge_y=detection2.corners[3][1]-detection2.corners[0][1]

		# Calculate dot product of each corner of the AprilTag square
            dot_prod_TR = top_edge_x*right_edge_x+top_edge_y*right_edge_y
            dot_prod_RB = right_edge_x*bottom_edge_x+right_edge_y*bottom_edge_y
            dot_prod_BL = bottom_edge_x*left_edge_x+bottom_edge_y*left_edge_y
            dot_prod_LT = left_edge_x*top_edge_x+left_edge_y*top_edge_y
            
		# Define angle approximation to be the maximum dot product value
		# normalized by the top edge length multiplied by a scalar factor
            angle_approx = max(abs(dot_prod_TR), abs(dot_product_LT))/(2*((top_edge_x/2)**2))*50
            print("[1] Angle Approximation: ", angle_approx)
            
		# Define estimated distance as the top edge length multiplied by 
		# angle approximation scaling divided by a scalar factor
            estimated_distance = (135/(abs(top_edge_x))*10*math.cos(math.radians(angle_approx)))
            estimated_distance = '{0:.4g}'.format(estimated_distance)
            print("[1] Estimated Distance: ", estimated_distance)
        
# Define the coordinates of the AprilTag highlighting overlay shown
# on the video feed
        overlay = frame // 2 + dimg[:, :, None] // 2
        overlay2 = frame2 // 2 + dimg2[:, :, None] // 2

	# Show the video feed
        cv2.imshow(window, overlay)
        cv2.imshow(window2, overlay2)

        k = cv2.waitKey(1)
 
        if k == 27:
            break

if __name__ == '__main__':
    main()
