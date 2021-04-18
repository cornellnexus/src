import numpy as np
import math
import matplotlib.pyplot as plt
from kinematics import limit_cmds, feedback_lin, integrate_odom
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
    r2d2 = Robot(-5,-10,math.pi/2)

    '''MOTION CONTROL'''
    goal1 = (5,10)
    goal2 = (2,1)
    goals = np.array([[5,5],[-5,5],[-5,-5],[5,-5]])
    # goals = [(-2.5,-5)]
    goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],[0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],[10,0],[10,-5],[10,-10]])


    #Larger epsilons means a larger turning radius
    EPSILON = 2
    # Used in limit_cmds
    MAX_V = .5
    ROBOT_RADIUS = 0.2

    curr_goal_ind = 0

    while curr_goal_ind < np.shape(goals)[0]:
    # for i in range(500):
        curr_goal = goals[curr_goal_ind, :]
        distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
            float(r2d2.state[1]) - curr_goal[1])

        while distance_away > 0.2:
            # print(curr_goal)
            # print(r2d2.state)
            # print(distance_away)
            cmd_v, cmd_w = feedback_lin(r2d2.state, curr_goal[0] - r2d2.state[0], \
                curr_goal[1] - r2d2.state[1], EPSILON)
            
            (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
            curr_x = r2d2.state[0]
            curr_y = r2d2.state[1]
            curr_theta = r2d2.state[2]
            
            r2d2.travel(limited_cmd_v, limited_cmd_w)
            
            distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
            float(r2d2.state[1]) - curr_goal[1])

        curr_goal_ind += 1

    # #Visit goal1, works correctly
    # for i in range(5000):
    #     cmd_v, cmd_w = feedback_lin(r2d2.state, goal1[0] - r2d2.state[0], \
    #         goal1[1] - r2d2.state[1], EPSILON)
    #     (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
    #     r2d2.travel(0.1 * limited_cmd_v, 0.1 * limited_cmd_w)

    # #Visit goal2, does not work, robot gets stuck at ~(2.5, 9)
    # for i in range(10000):
    #     x_error = goal2[0] - r2d2.state[0]
    #     y_error = goal2[1] - r2d2.state[1]
    #     cmd_v, cmd_w = feedback_lin(r2d2.state, x_error, y_error, EPSILON)
    #     curr_x = r2d2.state[0]
    #     curr_y = r2d2.state[1]
    #     curr_theta = np.degrees(r2d2.state[2])

        
    #     (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, 
    #     ROBOT_RADIUS)
    #     r2d2.travel(0.01 * limited_cmd_v, 0.01 * limited_cmd_w)

    '''PLOTTING'''
    plt.style.use('seaborn-whitegrid')
    x_coords = r2d2.truthpose[:,0]
    y_coords = r2d2.truthpose[:,1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')
    ax.plot(x_coords[0], y_coords[0], 'gx')
    ax.plot(goals[:,0], goals[:,1], 'rx')
# [(5,10),(2,1),(6,7),(4,7)]

    plt.xlim([-20, 20])
    plt.ylim([-20, 20])
    plt.show()
