import numpy as np
import math

from collections import deque
from engine.grid import Grid
from engine.kinematics import limit_cmds, feedback_lin, integrate_odom
from engine.pid_controller import PID
from engine.robot import Robot
 
 #TODO: documentation
class PathTraversal():
  def engine():
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
    print("REACHED THE END OF PATH TRAVERSAL")
