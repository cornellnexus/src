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

if __name__ == "__main__":
    # Initialize robot
    #xpos, ypos, heading
    r2d2 = Robot(-5,-10,math.pi/2)

    '''MOTION CONTROL'''
    #simulated noise added to robot's state 
    NOISE_RANGE = 0.1
    #TODO: Integrate Grid
    goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
    [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
    [10,0],[10,-5],[10,-10]])

    #Larger epsilons means a larger turning radius
    EPSILON = 0.2

    # Used in limit_cmds
    MAX_V = 0.5
    ROBOT_RADIUS = 0.2
    ALLOWED_DIST_ERROR = 0.5 # was 0.5, weird when 10 and everything 0.1
    TIME_STEP = 0.1 # is this the time by which we are going to sleep after each movement?

    loc_pid_x = PID(
        Kp=1, Ki=0.1, Kd=0.1, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    loc_pid_y = PID(
        Kp=1, Ki=0.1, Kd=0.1, target=0, sample_time=TIME_STEP, output_limits=(None, None)
    )

    curr_goal_ind = 0
    num_goals = np.shape(goals)[0]

    for curr_goal_ind in range(num_goals): # while queue (once we integrate Grid)
        
        curr_goal = goals[curr_goal_ind] # target coords
        predicted_state = r2d2.state # this will come from Kalman Filter

        # location error (in meters)
        distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
            float(predicted_state[1]) - curr_goal[1])

        while distance_away > ALLOWED_DIST_ERROR:
            r2d2.state[0] = np.random.normal(r2d2.state[0],NOISE_RANGE)
            r2d2.state[1] = np.random.normal(r2d2.state[1],NOISE_RANGE)

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

    '''PLOTTING'''
    plt.style.use('seaborn-whitegrid')
    x_coords = r2d2.truthpose[:,0]
    y_coords = r2d2.truthpose[:,1]
    fig, ax = plt.subplots()
    ax.plot(x_coords, y_coords, '-b')
    ax.plot(x_coords[0], y_coords[0], 'gx')
    ax.plot(goals[:,0], goals[:,1], 'rx')

    plt.xlim([-20, 20])
    plt.ylim([-20, 20])
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