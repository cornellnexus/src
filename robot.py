import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics import *
from collections import deque
import time

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
        self.truthpose = np.transpose(np.array([[x_pos],[y_pos],[heading]]))
        
        self.phase = init_mode
        self.is_sim = is_sim
        self.battery = init_charge
        self.waste_capacity = init_capacity

    def move_forward(self, dist, dt=1): 
        # Moves robot forward by distance dist
        # dist is in meters
        new_x = self.state[0] + dist * math.cos(self.state[2]) * dt
        new_y = self.state[0] + dist * math.sin(self.state[2]) * dt
        self.state[0] = np.round(new_x,3)
        self.state[1] = np.round(new_y,3)

        if self.is_sim:
            self.truthpose = np.append(self.truthpose,np.transpose(self.state), 0)

    def travel(self, dist, turn_angle):
        # Moves the robot with both linear and angular velocity
        self.state = np.round(integrate_odom(self.state, dist, turn_angle),3)
        if self.is_sim:
            self.truthpose = np.append(self.truthpose,np.transpose(self.state), 0)
    
    def turn(self, turn_angle, dt=1):
        # Turns robot, where turn_angle is given in radians
        self.state[2] = np.round(self.state[2] + (turn_angle * dt),3)
        if self.is_sim:
            self.truthpose = np.append(self.truthpose,np.transpose(self.state), 0)

    def get_state(self):
        return self.state

    def print_current_state(self):
        # Prints current robot state
        print('pos: ' + str(self.state[0:2]))
        print('heading: ' + str(self.state[2]))

if __name__ == "__main__":
    # Initialize robot
    r2d2 = Robot(0,0,math.pi/2)

    '''MOTION CONTROL'''
    goal1 = (5,10)
    goal2 = (2,1)
    goals = [(5,10),(2,1),(6,7),(4,7)]

    #Larger epsilons means a larger turning radius
    EPSILON = 0.2
    # Used in limit_cmds
    MAX_V = .4
    ROBOT_RADIUS = 0.2

    #Visit goal1, works correctly
    for i in range(5000):
        cmd_v, cmd_w = feedback_lin(r2d2.state, goal1[0] - r2d2.state[0], \
            goal1[1] - r2d2.state[1], EPSILON)
        (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
        r2d2.travel(0.1 * limited_cmd_v, 0.1 * limited_cmd_w)
    #Visit goal2, does not work, robot gets stuck at ~(2.5, 9)
    for i in range(10000):
        cmd_v, cmd_w = feedback_lin(r2d2.state, goal2[0] - r2d2.state[0], \
            goal2[1] - r2d2.state[1], EPSILON)
        (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
        r2d2.travel(0.01 * limited_cmd_v, 0.01 * limited_cmd_w)

    '''PLOTTING'''
    plt.style.use('seaborn-whitegrid')
    x_coords = r2d2.truthpose[:,0]
    y_coords = r2d2.truthpose[:,1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')

    plt.xlim([-20, 20])
    plt.ylim([-20, 20])
    plt.show()

    '''
    Below is an attempt at having the robot visit a series of goals, did not work
    '''

        # while len(goals) > 0:
    # # for i in range(500):
    #     curr_goal = goals.pop(0)
    #     distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
    #         float(r2d2.state[1]) - curr_goal[1])

    #     while distance_away > 0.1:
    #         print(curr_goal)
    #         print(r2d2.state)
    #         print(distance_away)
    #         cmd_v, cmd_w = feedback_lin(r2d2.state, curr_goal[0] - r2d2.state[0], \
    #             curr_goal[1] - r2d2.state[1], EPSILON)
            
    #         (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
    #         r2d2.travel(0.1 * limited_cmd_v, 0.1 * limited_cmd_w)

    #         distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
    #         float(r2d2.state[1]) - curr_goal[1])
