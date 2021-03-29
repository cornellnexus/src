from UserUtils import *
from grid import *
from collections import deque
import csv
import geopy
import gps #temporary import for update_step() placement for kalman filter
from commands import * 

print("under the imports")

"""
Test file - moving in a straight line to test GPS accuracy 
"""
def engine():
    print("TESTING: BEGINING TO TESTGPS.PY")
    longMin, longMax, latMin, latMax = -76.483682, -76.483682, 42.444250, 42.444416

    # Create graph object given longitute
    # and latitude coordinates from user input.
    # g = Grid(longMin, longMax, latMin, latMax)
    # queue = deque(g.traversal_path[:])

    #straightline path!
    queue = [(-76.483682, 42.444250), (-76.483682,42.444416)]

    target_node = queue.popleft()  # Next node to visit
    target_coords = target_node.get_coords()
    # update_step writes to CSV file,
    # returns GPS data in the form (lat,long)
    predicted_loc = gps.update_step()
    print("Predicted Location: " + str(predicted_loc))

    # distance_from_target <- get pythagerean distance from target in meters
    # must be in form latitude,longitude.
    # check distance is correct (order of coordinates in initialization):
    distance_from_target = \
        geopy.distance.distance(predicted_loc, target_coords).meters
    gps_noise_range = 3

    # while robot is too far away from target node
    while distance_from_target > gps_noise_range:
        print("DISTANCE FROM TARGET GREATER THAN NOISE RANGE")
        # move forward command; talk to electrical about moving
        go_forward() 
        print("MOVING FORWARD")
        predicted_loc = gps.update_step()
        print("GPS PREDICTED LOCATION: " + str(predicted_loc))
        distance_from_target = geopy.distance.distance(predicted_loc,target_node.get_coords()).meters
        print("DISTANCE FROM TARGET: " + str(distance_from_target) )
    stop()
    print("STOP")
    # We are currently at target node (next_node)
    print("Reached node at" + str(target_coords))


    ##########################################################################
    #COMMENT OUT TO TEST IF TURN RIGHT WORKS 
    # while distance_from_target > gps_noise_range:
    #     print("DISTANCE FROM TARGET GREATER THAN NOISE RANGE")
    #     # move forward command; talk to electrical about moving
    #     go_forward() 
    #     print("MOVING FORWARD")
    #     predicted_loc = gps.update_step()
    #     print("GPS PREDICTED LOCATION: " + str(predicted_loc))
    #     distance_from_target = geopy.distance.distance(predicted_loc,target_node.get_coords()).meters
    #     print("DISTANCE FROM TARGET: " + str(distance_from_target) )
    # print("TURNING RIGHT")
    # print("GPS PREDICTED LOCATION: " + str(predicted_loc))
    # turn_right()
    # print("FINISHING TURNING RIGHT")
    # print("GPS PREDICTED LOCATION: " + str(predicted_loc))
    # stop()
    # print("STOP")


    # Add support for turning L and R.
    # if target_coords[1] == g.true_max_lat:
    #     turn_right()
    #     print("Turn right")
    # elif target_coords[1] == g.true_min_lat:
    #     turn_left()
    #     print("Turn left")


if __name__ == "__main__":
    engine()

# engine = Engine()
# engine.run()
