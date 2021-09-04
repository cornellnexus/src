import numpy as np
import math
from engine.kinematics import integrate_odom

"""
Defines a dummy robot to be used for debugging
"""


class Robot:
    """
    Initializes the robot with the given position and heading.
    Parameters:
    state = Robot's state, np.array
        state contain's the robot's x position, y position, and heading
    phase = 'collect' when traversing through grid, 'return' when returning to 
        base station
    is_sim = False when running code with physical robot, True otherwise
    Preconditions:
    position is list of size 2
    heading is in range [0..359]
    """

    def __init__(self, x_pos, y_pos, heading, init_mode='collect', \
                 is_sim=True, init_charge=100, init_capacity=100):

        self.state = np.array([[x_pos], [y_pos], [heading]])
        self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
        self.is_sim = is_sim
        self.phase = init_mode
        self.battery = init_charge
        self.waste_capacity = init_capacity

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        self.state = np.round(integrate_odom(self.state, dist, turn_angle), 3)

        # if it is a simulation,
        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def move_forward(self, dist, dt=1):
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.state[0] + dist * math.cos(self.state[2]) * dt
        new_y = self.state[0] + dist * math.sin(self.state[2]) * dt
        self.state[0] = np.round(new_x, 3)
        self.state[1] = np.round(new_y, 3)

        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def turn(self, turn_angle, dt=1):
        # Turns robot, where turn_angle is given in radians
        clamp_angle = (self.state[2] + (turn_angle * dt)) % (2 * math.pi)
        self.state[2] = np.round(clamp_angle, 3)
        if self.is_sim:
            self.truthpose = np.append(self.truthpose, np.transpose(self.state), 0)

    def get_state(self):
        return self.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.state[0:2]))
        print('heading: ' + str(self.state[2]))
