from UserUtils import *
# from FileUtils import *
# from Graph import Graph
# from GUI import GUI
# from obstacle import *
from Grid import *
from collections import deque
import csv
import geopy
from rpi_read import *

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
        predicted_loc = update_step()

        # distance_from_target <- get pythogerean distance from target in meters
        # must be in form latitude,longitude.
        # check distance is correct (order of coordinates in initialization):
        distance_from_target = \
            geopy.distance.distance(predicted_loc, target_coords).meters
        gps_noise_range = 3

        # while robot is too far away from target node
        while distance_from_target > gps_noise_range + .1 or\
                distance_from_target < gps_noise_range - .1:
            # move forward command; talk to electrical about moving
            ser.write(b'MF')
            predicted_loc = update_step()
            distance_from_target = geopy.distance.distance(predicted_loc,
                                                           target_node.get_coords).meters

        # We are currently at target node (next_node)
        print("Reached node at" + str(target_coords))

        # Add support for turning L and R.
        if target_coords[1] == g.true_max_lat:
            ser.write(b'TR')
        elif target_coords[1] == g.true_min_lat:
            ser.write(b'TL')

# engine = Engine()
# engine.run()
