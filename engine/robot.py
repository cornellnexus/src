import numpy as np
import math
from engine.kinematics import integrate_odom, feedback_lin, limit_cmds
from engine.pid_controller import PID
from enum import Enum

"""
Defines a dummy robot to be used for debugging
"""

class Phase(Enum):
    SETUP = 1
    TRAVERSE = 2
    AVOID_OBSTACLE = 3
    RETURN = 4
    COMPLETE = 5


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

    def __init__(self, x_pos, y_pos, heading, epsilon, max_v, radius, is_sim=True, init_mode=1,
                 init_charge=100, init_capacity=100, time_step=0.1):

        self.state = np.array([[x_pos], [y_pos], [heading]])
        self.truthpose = np.transpose(np.array([[x_pos], [y_pos], [heading]]))
        self.is_sim = is_sim
        self.phase = Phase(init_mode)
        self.epsilon = epsilon
        self.max_velocity = max_v
        self.radius = radius
        self.battery = init_charge
        self.waste_capacity = init_capacity
        self.time_step = time_step

        self.loc_pid_x = PID(
            Kp=1, Ki=0, Kd=0, target=0, sample_time=self.time_step, output_limits=(None, None)
        )

        self.loc_pid_y = PID(
            Kp=1, Ki=0, Kd=0, target=0, sample_time=self.time_step, output_limits=(None, None)
        )

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

    def execute_setup(self):
        pass

    def execute_traversal(self, waypoints, allowed_dist_error):
        for curr_goal_ind in range(len(waypoints)):

            curr_goal = waypoints[curr_goal_ind].get_coords()  # target coords (meters)
            predicted_state = self.state  # this will come from Kalman Filter

            # location error (in meters)
            distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
                                       float(predicted_state[1]) - curr_goal[1])

            while distance_away > allowed_dist_error:
                # r2d2.state[0] = np.random.normal(r2d2.state[0],NOISE_RANGE)
                # r2d2.state[1] = np.random.normal(r2d2.state[1],NOISE_RANGE)

                x_error = curr_goal[0] - self.state[0]
                y_error = curr_goal[1] - self.state[1]

                x_vel = self.loc_pid_x.update(x_error)
                y_vel = self.loc_pid_y.update(y_error)

                # the x_vel and y_vel we pass into feedback_lin should be global. Are they?
                cmd_v, cmd_w = feedback_lin(predicted_state, x_vel, y_vel, self.epsilon)

                # clamping of velocities?
                (limited_cmd_v, limited_cmd_w) = limit_cmds(cmd_v, cmd_w, self.max_velocity, self.radius)

                self.travel(self.time_step * limited_cmd_v, self.time_step * limited_cmd_w)
                # sleep in real robot.

                # Get state after movement:
                predicted_state = self.state  # this will come from Kalman Filter
                # location error (in meters)
                distance_away = math.hypot(float(predicted_state[0]) - curr_goal[0], \
                                           float(predicted_state[1]) - curr_goal[1])

        self.phase = Phase.COMPLETE

    def execute_avoid_obstacle(self):
        pass

    def execute_return(self):
        pass