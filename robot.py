import numpy as np
import math
from kinematics import integrate_odom

"""
Defines a robot to be used to track state variables as well as for debugging
"""
class Robot:
    """
    Initializes the robot with the given position and heading.
    Parameters:
    state = Robot's state, np.array
    phase = 'collect' when traversing through grid, 'return' when returning to \
        base station
    is_sim = False when running code with physical robot, True otherwise
    Preconditions:
    heading is in radians, where 0 = East and pi/2 = North
    """
    def __init__(self, x_pos, y_pos, heading, init_mode = 'collect', \
        is_sim = True, init_charge = 100, init_capacity = 100): 
        self.state = np.array([[x_pos],[y_pos],[heading]])
        self.phase = init_mode
        self.is_sim = is_sim
        self.battery = init_charge
        self.waste_capacity = init_capacity

    def move_forward(self, dist, dt=0.1): 
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.state[0] + dist * math.cos(self.state[2]) * dt
        new_y = self.state[0] + dist * math.sin(self.state[2]) * dt
        self.state[0] = new_x
        self.state[1] = new_y

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        self.pose = integrate_odom(self.pose, dist, turn_angle)
    
    def turn(self, turn_angle, dt=0.1):
        # Turns robot, where turn_angle is given in radians
        self.state[2] += (turn_angle * dt)

    def get_state(self):
        return self.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.state[0:2]))
        print('heading: ' + str(self.state[2]))
