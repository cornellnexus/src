import numpy as np
from numpy.linalg import inv
from engine.kinematics import integrate_odom
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

class LocalizationEKF:
    """
    An implementation of the EKF localization for the robot.

    INSTANCE ATTRIBUTES:
        # robot_width: the width of the robot (meters)

        # mu: the mean of the robot state distribution, in the form [[x],[y],[z]]. [3x1 np array]

        # sigma: the standard deviation of the robot state distribution. [3x3 np array]

        # Q: the process noise covariance matrix [3x3 np array].
            Note: The dimensions of Q are n-by-n, where n is the degrees of freedom of the robot state. (n=3)

        # R: measurement noise covariance matrix [3x3 np array]
            Note: The dimensions of R are k-by-k, where k is the degrees of freedom of the measurement. (k=3)
    """

    def __init__(self, init_mu, init_sigma, robot_width):
        self.robot_width = robot_width
        self.mu = init_mu
        self.sigma = init_sigma
        # process noise matrix
        self.Q = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
        self.R = np.array([[5, 0, 0], [0, 5, 0], [0, 0, 5]]
                          )  # measurement noise matrix

    @staticmethod
    def get_predicted_state(pose, control):
        """
        g function
        Given a robot pose at time step t, and the controls of the robot at time t-1,
        returns the predicted robot pose at the next time step t+1 in the inertial frame based on odometry measurements.
        [3-by-1 Numpy array].

        Parameters:
        -----------
        # pose: the 3-by-1 Numpy array representing the robot's pose. [x; y; theta]
        # control: the 2-by-1 Numpy array representing the robot's odometry measurements. [d, phi],
                   where d = distance traveled in the time step and phi = angle turned in the time step
        """
        return integrate_odom(pose, control[0], control[1])

    @staticmethod
    def get_g_jac(pose, control):
        """
        G Jacobian
        Returns the Jacobian of the g function.

        Parameters:
        ----------
        # pose: robot's state
        # control: 2-by-1 vector of [d, phi]
        """
        theta_prev = pose[2]
        d = control[0]
        phi = control[1]
        if phi == 0:
            G = np.array([[1, 0, - d * math.sin(theta_prev)],
                          [0, 1, d*math.cos(theta_prev)],
                          [0, 0, 1]])
        else:
            theta_t = [0, 0, 1]
            x_t = [1, 0, d / phi *
                   (math.cos(theta_prev + phi) - math.cos(theta_prev))]
            y_t = [0, 1, - d / phi *
                   (-math.sin(theta_prev + phi) + math.sin(theta_prev))]
            G = np.array([x_t, y_t, theta_t])
        return G

    @staticmethod
    def get_expected_measurement(pose):
        """
        h function
        Returns the expected measurement of the robot, given the robot's current state. [3-by-1 Numpy array], where
        k = 3 dimensions of measurements

        Parameters:
        # pose: robot's state
        """
        # First get data, convert data to how it relates to our pose
        # GPS -> meters away from origin
        # o           o          o X           o

        expected_measurement = pose
        return expected_measurement

    def get_h_jac(self, pose):
        """
        H Jacobian
        Returns the Jacobian of the h function.

        Parameters:
        # pose: robot's state
        """
        return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def get_arc_lengths(self, left_motor_revolutions, right_motor_revolutions):
        """
        Returns the arc_lengths of the left side of the robot and the right side of the robot 
        given the angular velocity of the left side and angular velocity of the right side. 

        left_side_w: angular velocity of the motors on the left side of the robot
        right_side_w: angular velocity of the motors on the right side of the robot
        """
        # TODO: from electrical: from motors, number of revolutions
        # TODO: from number of revolutions, using the circumference of the wheels, figure out the distance traveled 
        # Note: this was put on hold Spring '23 because Professor advised that getting velocity
        # in this fashion was not that accurate 

        # THESE ARE TEMPORARILY PLACE HOLDERS. 
        # We will get these values from an electrical function
        left_arc_length = left_motor_revolutions # this will be some sort of relation with the wheel circumference
        right_arc_length = right_motor_revolutions # this will be some sort of relation with the wheel circumference

        return [left_arc_length, right_arc_length]


    def get_controls(self, arc_lengths): 
        """
        Returns phi, the angle changed in a timestep and d, the displacement traveled in a timestep.

        Given from electrical: arc_lengths: a tuple such that the first entry is the 
        left-side arc distance and the second entry is the right-side arc distance
        """

        left_side_arc_length = arc_lengths[0]
        right_side_arc_length = arc_lengths[1]

        try:
            turn_radius = left_side_arc_length * self.robot_width / (right_side_arc_length - left_side_arc_length)
        except:
            turn_radius = 0
        
        phi = (right_side_arc_length - left_side_arc_length) / self.robot_width
        d = 2 * ((turn_radius + (self.robot_width/2)) * math.sin(phi / 2))
        return [d, phi]

    def predict_step(self, arc_lengths):
        """
        Returns the distribution of the robot's state after the prediction step of the EKF, based
        on the robot's controls.

        Let arc_lengths be a tuple such that it represents
        (left-side-length-traveled, right-side-length-traveled) 

        [mu_bar, sigma_bar]
        """
        controls = self.get_controls(arc_lengths) # [d, phi]

        jac_G = self.get_g_jac(self.mu, controls)

        mu_bar = self.get_predicted_state(self.mu, controls)
        sigma_bar = jac_G * self.sigma * np.transpose(jac_G) + self.Q

        return mu_bar, sigma_bar

    def update_step(self, mu_bar, sigma_bar, measurement):
        """
        Returns the distribution of the robot's state after the update step of the EKF, based
        on the robot's sensor measurements.
        [mu, sigma]
        TODO: Fix this specification. update_step is a procedure and does not return anything.
        """

        jac_H = self.get_h_jac(mu_bar)

        kalman_gain = (sigma_bar * np.transpose(jac_H) * inv(jac_H * sigma_bar * np.transpose(jac_H) + self.R))

        #JULIE NOTE: I think expected_measurement == mu_bar, since mu_bar represents
        #our prediction of the robot's state based on the controls. 
        expected_measurement = self.get_expected_measurement(mu_bar) 

        self.mu = mu_bar + (kalman_gain @ (measurement - expected_measurement))
        kh = kalman_gain * jac_H
        self.sigma = (np.eye(np.size(kh, 0)) - kh) @ sigma_bar

   