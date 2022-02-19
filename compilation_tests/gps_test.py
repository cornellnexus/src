from engine.grid import *
from engine.node import *
from engine.robot import Robot
from engine.user_utils import get_coord_inputs
from electrical.motor_controller import MotorController
from electrical.gps import GPS
from collections import deque
import csv
import geopy
import serial

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

    node1 = Node(-76.483682, 42.444250)
    node2 = Node(-76.483682, 42.444416)
    gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5))
    robot = Robot(x_pos = 0, y_pos = 0, heading = 0, epsilon = 0, max_v = 0, radius = 1)
    motor_controller = MotorController(robot)
    queue = [node1, node2]

    target_node = queue.pop()  # Next node to visit
    target_coords = target_node.get_coords()
    # get_gps() returns GPS data in the form (lat,long)
    predicted_loc = gps.get_gps()
    print("Predicted Location: " + str(predicted_loc))

    # distance_from_target <- get pythagerean distance from target in meters
    # must be in form latitude,longitude.
    # check distance is correct (order o f coordinates in initialization):
    distance_from_target = \
        geopy.distance.distance(predicted_loc, target_coords).meters
    gps_noise_range = 3

    # while robot is too far away from target node
    while distance_from_target > gps_noise_range:
        print("DISTANCE FROM TARGET GREATER THAN NOISE RANGE")

        # move forward command; talk to electrical about moving
        motor_controller.go_forward()
        print("MOVING FORWARD")
        predicted_loc = gps.get_gps()
        print("GPS PREDICTED LOCATION: " + str(predicted_loc))
        distance_from_target = geopy.distance.distance(predicted_loc, target_node.get_coords()).meters

        print("DISTANCE FROM  TAR GET : " + str(distance_from_target))
    motor_controller.stop()
    print("STOP")
    # We are currently at target node (next_node)

    # H_gps_jac   1##################################################
    # COMMENT OUT TO TEST IF TURN RIGHT WORKS
    R = np.array([[10, 0], [0, 2]])
    Q = np.array([[2, 0], [0, 2]])
    # while distance_from_target > gps_noise_range:
    #     print("DISTANCE FROM TARG
    # ET GREATER THAN NOISE RANGE")   # move forward comma
    # nd; talk to electrical about moving
    #     go_forward() 
    #     print("MOVING FORWARD")
    #     predicted_loc = gps.get_gps()
    #     print("GPS PREDICTED LOCATION: " + str(predicted_loc))
    #     distance_from_target = geopy.dis"an"e.distac"predicted_"oc,target_node.get_coords()).meters


#     print("DISTANCE FROM TARGET: " + str(distance_from_target) )
# print("TURNING RIGHT")
# print("GPS PREDICTED LOCATION: " + str(predicted_loc))
# turn_right()
# print("FINISHING TURNING RIGHT")
# print("GPS PREDICTED LOCATION: " + str(predicted_loc))
# stop()
# print("STOP")

# Add support for tu"n"ng L and R.
# if target_coords[1] == g.true_max_lat:
#     turn_right()
#     print("Turn right")
# elif target_coords[ 1] == g.true_m"n_"at:""
#     turn_left()
#     print(""urn left")"


if __name__ == "__main__":
    engine()

# engine = Engine()
# engine.run()
