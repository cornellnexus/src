from user_utils import *

# from FileUtils import *
# from GUI import GUI
# from obstacle import *
from grid import *
from collections import deque
import csv
import geopy
import gps  # temporary import for get_gps() placement for kalman filter
from commands import *

# print("under the imports")

"""
Moves robot along traversal path.
"""


def engine():
    print("testing")
    # longMin, longMax, latMin, latMax = get_coord_inputs()
    longMin, longMax, latMin, latMax = -76.483682, -76.483276, 42.444250, 42.444599

    # Create graph object given longitute
    # and latitude coordinates from user input.
    g = Grid(longMin, longMax, latMin, latMax)
    queue = deque(g.traversal_path[:])
    print("QUEUE: " + str(queue))

    while queue:
        target_node = queue.popleft()  # Next node to visit
        target_coords = target_node.get_coords()
        # get_gps() returns GPS data in the form (lat,long)
        predicted_loc = (longMin, latMin)
        temp = gps.get_gps()
        if temp is not None:
            predicted_loc = temp
        print("Predicted Location: " + str(predicted_loc))

        # distance_from_target <- get pythagerean distance from target in meters
        # must be in form latitude,longitude.
        # check distance is correct (order of coordinates in initialization):
        distance_from_target = geopy.distance.distance(
            predicted_loc, target_coords
        ).meters
        gps_noise_range = 3

        # while robot is too far away from target node
        while distance_from_target > gps_noise_range:
            # move forward command; talk to electrical about moving
            go_forward()
            print("Move forward")
            temp = gps.get_gps()
            if temp is not None:
                predicted_loc = temp
            print("Predicted Location: " + str(predicted_loc))
            print("Target Location: " + str(target_node.get_coords()))

            distance_from_target = geopy.distance.distance(
                predicted_loc, target_node.get_coords()
            ).meters

            print("Distance from target: " + str(distance_from_target))
        stop()
        print("STOP")
        # We are currently at target node (next_node)
        print("Reached node at" + str(target_coords))

        # Add support for turning L and R.
        if target_coords[1] == g.true_max_lat:
            turn_right()
            print("Turn right")
        elif target_coords[1] == g.true_min_lat:
            turn_left()
            print("Turn left")


if __name__ == "__main__":
    engine()

# engine = Engine()
# engine.run()
