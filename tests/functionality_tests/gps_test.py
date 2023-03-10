from engine.grid import *
from engine.node import *
from engine.robot import Robot
from engine.user_utils import get_coord_inputs
from electrical.motor_controller import BasicMotorController, MotorController
from engine.robot_state import Robot_State
from electrical.gps import GPS
from collections import deque
import csv
import geopy
import serial

"""
Test file - moving in a straight line to test GPS accuracy 
"""


def engine():
    print("TESTING: BEGINING TO TESTGPS.PY")
    longMin, longMax, latMin, latMax = -76.483682, -76.483682, 42.444250, 42.444416

    # Create graph object given longitute and latitude coordinates from user input.
    # g = Grid(longMin, longMax, latMin, latMax)
    # queue = deque(g.traversal_path[:])

    node1 = Node(-76.483682, 42.444250)
    node2 = Node(-76.483682, 42.444416)
    gps = GPS(serial.Serial('/dev/ttyACM0', 19200, timeout=5))
    robot_state = Robot_State(x_pos=0, y_pos=0, heading=0, epsilon=0, max_velocity=0, radius=1)
    robot = Robot(robot_state)
    motor_controller = BasicMotorController(robot)
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


if __name__ == "__main__":
    engine()

# engine = Engine()
# engine.run()
