import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from kinematics import limit_cmds, feedback_lin, integrate_odom
import time
import random
from robot import Robot
from pid_controller import PID
from grid import Grid
from collections import deque

'''PLOTTING'''
def waypoints_to_array(waypoints):
    """
    Tranforms a list of Node objects to a 1D np array of coordinates to be plotted
    for easier plotting. 
    """
    n = len(waypoints)
    waypoints_arr = np.empty([n,2])
    for i in range(n):
        waypoints_arr[i,:] = np.asarray(waypoints[i].get_coords())
    return waypoints_arr 

if __name__ == "__main__":
    # Initialize robot
    r2d2 = Robot(0,0,math.pi/2)

    '''MOTION CONTROL'''
    # Grid: Engineering Quad
    g = Grid(42.444250, 42.444599, -76.483682, -76.483276)

    # pass in 'full' to get full traversal path
    waypoints = g.get_waypoints('borders')

    '''MOTION CONTROL'''
    #simulated noise added to robot's state 
    NOISE_RANGE = .1

    goals = np.array(g.gps_waypoints)

    #Larger epsilons means a larger turning radius
    EPSILON = 0.2

    # Used in limit_cmds
    MAX_V = 0.5
    ROBOT_RADIUS = 0.2
    ALLOWED_DIST_ERROR = 0.5 # was 0.5, weird when 10 and everything 0.1
    TIME_STEP = 0.1 # is this the time by which we are going to sleep after each movement?

    loc_pid_x = PID(
        Kp=1, Ki=0, Kd=0, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    loc_pid_y = PID(
        Kp=1, Ki=0, Kd=0, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    curr_goal_ind = 0
    num_goals = len(waypoints)

    for curr_goal_ind in range(num_goals): # TODO: while queue: should we switch to this format?
        
        curr_goal = waypoints[curr_goal_ind].get_coords() # target coords (meters)
        predicted_state = r2d2.state # this will come from Kalman Filter

        # location error (in meters)
        distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
            float(predicted_state[1]) - curr_goal[1])

        while distance_away > ALLOWED_DIST_ERROR:
            # r2d2.state[0] = np.random.normal(r2d2.state[0],NOISE_RANGE)
            # r2d2.state[1] = np.random.normal(r2d2.state[1],NOISE_RANGE)

            x_error = curr_goal[0] - r2d2.state[0]
            y_error = curr_goal[1] - r2d2.state[1]

            x_vel = loc_pid_x.update(x_error)
            y_vel = loc_pid_y.update(y_error)

            # the x_vel and y_vel we pass into feedback_lin should be global. Are they?
            cmd_v, cmd_w = feedback_lin(predicted_state, x_vel, y_vel, EPSILON)
            
            #clamping of velocities?
            (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
            
            r2d2.travel(TIME_STEP * limited_cmd_v, TIME_STEP * limited_cmd_w)
            # sleep in real robot.

            # Get state after movement:
            predicted_state = r2d2.state # this will come from Kalman Filter
            # location error (in meters)
            distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
                float(predicted_state[1]) - curr_goal[1])

        # Turning? Heading pid?


    
    def get_plot_boundaries(meters_grid,delta):
        """
        Given some grid to be plotted, and a delta value, returns the desired 
        x limits and y limits for the plot.
        """
        size = np.shape(meters_grid)
        min_coords = meters_grid[0,0].get_coords()
        max_coords = meters_grid[size[0]-1, size[1]-1].get_coords()
        xlim = [min_coords[0]-delta, max_coords[0]+delta]
        ylim = [min_coords[1]-delta, max_coords[1]+delta]
        return xlim,ylim

    plt.style.use('seaborn-whitegrid')
    x_coords = r2d2.truthpose[:,0]
    y_coords = r2d2.truthpose[:,1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')
    ax.plot(x_coords[0], y_coords[0], 'gx')
    goals = waypoints_to_array(waypoints)
    ax.plot(goals[:,0], goals[:,1], 'rx')

    xbounds,ybounds = get_plot_boundaries(g.meters_grid,5)
    plt.xlim(xbounds)
    plt.ylim(ybounds)
    # plt.show()

    circle_patch = plt.Circle((5, 5), 1, fc="green")
    wedge_patch = patch.Wedge(
        (5, 1), 3, 100, 80, animated=True, fill=False, width=2, ec="g", hatch="xx"
    )

    def init():
        circle_patch.center = (0, 0)
        ax.add_patch(circle_patch)
        # ax.add_patch(arc_patch)
        ax.add_patch(wedge_patch)
        return circle_patch, wedge_patch

    def animate(i):
        x_coord = r2d2.truthpose[i,0]
        y_coord = r2d2.truthpose[i,1]
        circle_patch.center = (x_coord, y_coord)
        wedge_patch.update({"center": [x_coord, y_coord]})
        wedge_patch.theta1 = np.degrees(r2d2.truthpose[i,2]) - 10
        wedge_patch.theta2 = np.degrees(r2d2.truthpose[i,2]) + 10

        # print(wedge_patch.theta1, wedge_patch.theta2)
        # print(wedge_patch.center)
        return circle_patch, wedge_patch

    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=np.shape(r2d2.truthpose)[0], interval=20, blit=True
    )

    plt.show()



def simulation(robot, noise, goals, kp, ki, kd):
    #Larger epsilons means a larger turning radius
    EPSILON = 0.2

    # Used in limit_cmds
    MAX_V = 0.5
    ROBOT_RADIUS = 0.2
    ALLOWED_DIST_ERROR = 0.5 # was 0.5, weird when 10 and everything 0.1
    TIME_STEP = 0.1 # is this the time by which we are going to sleep after each movement?

    loc_pid_x = PID(
        Kp=kp, Ki=ki, Kd=kd, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    loc_pid_y = PID(
        Kp=kp, Ki=ki, Kd=kd, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    curr_goal_ind = 0
    num_goals = np.shape(goals)[0]
    targets_visited = [False] * num_goals #is an array of booleans telling us if the goal node at the index has been traveled to

    for curr_goal_ind in range(num_goals): # while queue (once we integrate Grid)
        
        curr_goal = goals[curr_goal_ind] # target coords
        predicted_state = robot.state # this will come from Kalman Filter

        # location error (in meters)
        distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
            float(predicted_state[1]) - curr_goal[1])

        while distance_away > ALLOWED_DIST_ERROR:
            robot.state[0] = np.random.normal(robot.state[0],noise)
            robot.state[1] = np.random.normal(robot.state[1],noise)

            x_error = curr_goal[0] - robot.state[0]
            y_error = curr_goal[1] - robot.state[1]

            x_vel = loc_pid_x.update(x_error)
            y_vel = loc_pid_y.update(y_error)

            # the x_vel and y_vel we pass into feedback_lin should be global. Are they?
            cmd_v, cmd_w = feedback_lin(predicted_state, x_vel, y_vel, EPSILON)
            
            #clamping of velocities?
            (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
            
            robot.travel(TIME_STEP * limited_cmd_v, TIME_STEP * limited_cmd_w)
            # sleep in real robot.

            # Get state after movement:
            predicted_state = robot.state # this will come from Kalman Filter
            # location error (in meters)
            distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
                float(predicted_state[1]) - curr_goal[1])
        targets_visited[curr_goal_ind] = True
    return targets_visited
    