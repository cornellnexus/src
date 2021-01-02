from UserUtils import *
# from FileUtils import *
# from GUI import GUI
# from obstacle import *
from grid import *
from collections import deque
import csv
import geopy
from rpi_read import *
import gps #temporary import for update_step() placement for kalman filter
from commands import * 

if __name__ == "__main__":
    longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()

    # Create graph object given longitute
    # and latitude coordinates from user input.
    g = Grid(longMin, longMax, latMin, latMax)
    queue = deque(g.traversal_path[:])

    while queue:
        target_node = queue.popleft()  # Next node to visit
        target_coords = target_node.get_coords()
        # update_step writes to CSV file,
        # returns GPS data in the form (lat,long)
        predicted_loc = gps.update_step()

        # distance_from_target <- get pythagerean distance from target in meters
        # must be in form latitude,longitude.
        # check distance is correct (order of coordinates in initialization):
        distance_from_target = \
            geopy.distance.distance(predicted_loc, target_coords).meters
        gps_noise_range = 3

        # while robot is too far away from target node
        while distance_from_target > gps_noise_range:
            # move forward command; talk to electrical about moving
            #moveForward() 
            print("Move forward")
            predicted_loc = update_step()
            distance_from_target = geopy.distance.distance(predicted_loc,
                                                           target_node.get_coords).meters
        #stop()
        print("STOP")
        # We are currently at target node (next_node)
        print("Reached node at" + str(target_coords))

        # Add support for turning L and R.
        if target_coords[1] == g.true_max_lat:
<<<<<<< HEAD
            # Make sure it stops rotating after 90 degrees!
            print("Turn right")
        elif target_coords[0] != g.true_min_long and target_coords[1] == g.true_min_lat:
=======
            # turnRight()
            print("Turn right")
        elif target_coords[1] == g.true_min_lat:
            # turnLeft()
>>>>>>> d18be93fa3efdff664481140b4f6bcafc9920bc9
            print("Turn left")

# engine = Engine()
# engine.run()
