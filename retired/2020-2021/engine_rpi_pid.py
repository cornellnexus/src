from collections import deque
import numpy as np
import random
import time
import math
import matplotlib.pyplot as plt

"""Simulates a Robot for debugging purposes"""


class Robot:
    """Initializes the robot with the given position and heading.
    Parameters:
    position: A 2-index array representing x and y position of robot
    heading: Angle robot is facing from North 
    """

    def __init__(self, position=[0, 0], heading=90):
        self.pos = position
        self.heading = heading

    """Randomly generates noise"""

    def get_noise(self):
        return np.random.uniform(-.02, .02)

    """Psuedo to_String method to print position and heading of robot."""

    def print_current_state(self):
        print('pos: ' + str(self.pos))
        print('heading: ' + str(self.heading))
        # time.sleep(0.1) 
        #  

    """Moves the robot forward. 
    If the robot is facing West or East, y position of robot is updated. 
    If the robot is facing North, x position of robot is updated.
    """

    def move_forward(self):
        if self.heading == 90:
            self.pos[1] = round(self.pos[1] + .1, 3)
            # self.pos[1] = round(self.pos[1] + .1 + self.get_noise(),3)
        elif self.heading == 270:
            self.pos[1] = round(self.pos[1] - .1, 3)
            # self.pos[1] = round(self.pos[1] - .1 + self.get_noise(),3)
        elif self.heading == 0:
            self.pos[0] = round(self.pos[0] + .1, 3)
            # self.pos[0] = round(self.pos[0] + .1 + self.get_noise(),3)
        self.print_current_state()

    """Turns the robot left. Updates the heading of the robot accordingly."""

    def turn_left(self):
        self.heading += 90
        self.heading = self.heading % 360
        self.print_current_state()

    """Turns the robot right. Updates the heading of the robot accordingly."""

    def turn_right(self):
        self.heading -= 90
        self.heading = self.heading % 360
        self.print_current_state()

    def get_position(self):
        return self.pos


# Create graph object given longitute
# and latitude coordinates from user input.
# g = Grid(longMin, longMax, latMin, latMax)
# Create a grid from 0..10 and 0..10
"""Returns a simulated traversal path of nodes for the robot to follow."""


def generate_nodes():
    origin = (0, 0)
    traversal_path = []
    for i in range(10):
        for j in range(10):
            if i % 2 == 0:
                node = origin[0] + i, origin[1] + j
                # node = Node(origin[0] + j, origin[1] + i)
                traversal_path.append(node)
            elif i % 2 == 1:
                # lat_pos = origin[0] + (9 - j)
                # long_pos = origin[1] + i
                y = origin[1] + (9 - j)
                x = origin[0] + i
                # node = Node(lat_pos, long_pos)
                node = (x, y)
                traversal_path.append(node)
    return traversal_path


"""Graphs the traversal path of the robot using matplotlib."""


def graph_traversal_path(traversal_path):
    print(traversal_path)
    xlist = []
    ylist = []

    for node in traversal_path:
        xlist.append(node[0])
        ylist.append(node[1])

    plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
    plt.ylim(min(ylist) - 1, max(ylist) + 1)
    plt.xlim(min(xlist) - 1, max(xlist) + 1)
    plt.show()
    plt.close()
    # plt.clf()
    # plt.cla()
    # plt.close()


"""Returns the distance given two (x,y) pairs. 
In our case, we are looking for the distance between the target (x,y) position
and our predicted (x,y) position."""


def get_distance(x_targ, x_pred, y_targ, y_pred):
    return math.sqrt((x_targ - x_pred) ** 2 + (y_targ - y_pred) ** 2)


"""Using simulated robot and traversal path, tests the functionality of the 
PID controllers, given simulated noise on received data."""


def engine_pid_main():
    # Initilize location PID controller and heading PID controller
    loc_pid = PID(Kp=0.02, Ki=0.005, Kd=0.0, target=0,
                  sample_time=0.01, output_limits=(None, None))
    head_pid = PID(Kp=0.02, Ki=0.005, Kd=0.0, target=0,
                   sample_time=0.01, output_limits=(None, None))

    # Generates traversal path
    g = generate_nodes()

    # List converted to a queue for constant append/pop operations
    queue = deque(g)
    r = Robot()

    # While the queue is not empty
    while queue:

        # Store the next node to visit from the grid and make it our target
        target_coords = queue.popleft()

        # Returns "GPS data" in the form (lat,long). For now, this is simulated.
        predicted_loc = r.get_position()
        predicted_head = r.get_heading()

        # distance should be close to 1 (??? explain pls) @jess/@yagmur?
        # # TODO: confirm location error = pythagorean or 2d? 
        # Error between location of target and predicted robot location
        location_error = \
            get_distance(target_coords[0], predicted_loc[0], \
                         target_coords[1], predicted_loc[1])
        allowed_error = 0  # TODO: measure this

        # While the robot is too far away from target node
        while location_error > allowed_error:
            # Set the target location for the location PID controller
            loc_pid = loc_pid.set_target(target_coords)
            # Set the target heading for the heading PID controller
            head_pid = head_pid.set_target(0)

            # Receive the velocity from the location PID correction
            vel = loc_pid.update(predicted_loc)
            # Receive the angular velocity from the heading PID correction
            ang_vel = head_pid.update(predicted_head)

            # Move the robot forward
            # TODO: talk to electrical about moving
            # TODO: send angular_velocity and velocity to electrical
            r.move_forward(vel, ang_vel)

            # Robot sleeps
            sleep(loc_pid.get_sample_time())

            # TODO: Get current location from Kalman Filter
            predicted_loc = r.get_position()
            # Updates the location error
            location_error = \
                get_distance(target_coords[0], predicted_loc[0], \
                             target_coords[1], predicted_loc[1])

            # We have reached the target node
        # Stop the robot and sleep
        r.stop(vel=0, ang_vel=0)
        sleep(robot_stop_time)

        # Turning Left and Right
        # If the robot is on the 'top' of its traversal path,
        # we want to turn right
        if target_coords[1] == 9:
            print("Turning right")
            next_target_coords = queue.peek()
            if target_coords[0] < next_target_coords[0]:
                target_angle = 0
            elif target_coords[0] == next_target_coords[0]:
                target_angle = 270
            print("Should not be here, faulty angle logic")
        # If the robot is at the 'bottom' of its traversal path,
        # we want to turn left
        elif target_coords[0] != 0 and target_coords[1] == 0:
            print("Turning left")
            next_target_coords = queue.peek()
            if target_coords[0] < next_target_coords[0]:
                target_angle = 0
            elif target_coords[0] == next_target_coords[0]:
                target_angle = 90

        # Calculates error in heading
        angle_error = abs(target_angle - r.heading)
        allowed_error = 2  # TODO: Measure this

        # While the heading is not corrected
        while angle_error > allowed_error:
            # Set the target angle of the heading PID controller
            head_pid.set_target(target_angle)

            # Receive the angular velocity from the heading PID correction
            angular_velocity = head_pid.update(angle_error)

            # TODO: send angular_velocity to electrical and (velocity = 0)

            # The robot turns
            sleep(head_pid.get_sample_time())

            # TODO: Get current heading from Kalman filter
            # r.set_heading = Kalman filter output

            # Updates the angle error
            angle_error = abs(target_angle - r.heading)

    # Plots the traversal path
    # plot_path(g)
    print("Reached end of traversal path!")


"""Plots the traversal path of the robot onto a GUI using matplotlib
    Parameters: traversal_path is the array representing the traversal path"""


def plot_path(traversal_path):
    xlist = []
    ylist = []
    for node in traversal_path:
        xlist.append(node[0])
        ylist.append(node[1])
    plt.plot(xlist, ylist, 'ro', markerfacecolor='blue')
    plt.ylim(min(ylist) - 1, max(ylist) + 1)
    plt.xlim(min(xlist) - 1, max(xlist) + 1)
    plt.show()

    xlist2 = []
    ylist2 = []
    for node in history:
        xlist2.append(node[0])
        ylist2.append(node[1])
    plt.plot(xlist2, ylist2, 'bx')
    plt.ylim(min(ylist2) - 1, max(ylist2) + 1)
    plt.xlim(min(xlist2) - 1, max(xlist2) + 1)
    plt.show()
    plt.close()

    # for node in history:
    #     print("Plotting node")
    #     plt.plot(node[0],node[1],marker='x',markerfacecolor='red')
    #     time.sleep(.1)
    #     plt.show()
    # plt.close()
