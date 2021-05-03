import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
from kinematics import limit_cmds, feedback_lin, integrate_odom
import time
import random
from robot import Robot

if __name__ == "__main__":
    # Initialize robot
    #xpos, ypos, heading
    r2d2 = Robot(-5,-10,math.pi/2)

    '''MOTION CONTROL'''
    goal1 = (5,10)
    goal2 = (2,1)
    goals = np.array([[5,5],[-5,5],[-5,-5],[5,-5]])
    # goals = [(-2.5,-5)]
    goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],[0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],[10,0],[10,-5],[10,-10]])

    #simulated noise added to robot's state 
    NOISE_RANGE = .2

    #Larger epsilons means a larger turning radius
    EPSILON = 2
    # Used in limit_cmds
    MAX_V = .5
    ROBOT_RADIUS = 0.2

    ALLOWED_DIST_ERROR = 0.5
    TIME_STEP = 0.1

    curr_goal_ind = 0

    while curr_goal_ind < np.shape(goals)[0]:
    # for i in range(500):
        curr_goal = goals[curr_goal_ind, :]
        #add noise to current r2d2 position reading

        #TODO: currently if the noise range is super big, you can see the robot 'jumping' over some areas 
  
        distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
            float(r2d2.state[1]) - curr_goal[1])

        while distance_away > ALLOWED_DIST_ERROR:

            #TODO: produce bellcurve error
            #match expected error of the gps
            #rn its all in meters
            # r2d2.state[0] += random.uniform(-.1,.1) 
            # r2d2.state[1] += random.uniform(-.1,.1)
            # print("NORMAL VALUE STATE 0: " + str(r2d2.state[0]))
            # print("USING NORMAL FUNCTION VALUE STATE 0: " + str(np.random.normal(r2d2.state[0],0.1)))
            r2d2.state[0] = np.random.normal(r2d2.state[0],NOISE_RANGE)
            # print("NORMAL VALUE STATE 1: " + str(r2d2.state[1]))
            # print("USING NORMAL FUNCTION VALUE STATE 1: " + str(np.random.normal(r2d2.state[1],0.1)))
            r2d2.state[1] = np.random.normal(r2d2.state[1],NOISE_RANGE)


            # print(curr_goal)
            # print(r2d2.state)
            # print(distance_away)
            #velcoity and angular velocity
            cmd_v, cmd_w = feedback_lin(r2d2.state, curr_goal[0] - 
            r2d2.state[0], \
                curr_goal[1] - r2d2.state[1], EPSILON)
            
            #clamping of velocities?
            (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, MAX_V, ROBOT_RADIUS)
            
            r2d2.travel(TIME_STEP * (limited_cmd_v), TIME_STEP * (limited_cmd_w))
            
            distance_away = math.hypot(float(r2d2.state[0]) - curr_goal[0], \
            float(r2d2.state[1]) - curr_goal[1])

        curr_goal_ind += 1

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