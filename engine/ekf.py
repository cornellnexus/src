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
        # mu: the mean of the robot state distribution, in the form [[x],[y],[z]]. [3x1 np array]

        # sigma: the standard deviation of the robot state distribution. [3x3 np array]

        # R: the process noise covariance matrix [3x3 np array].
            Note: The dimensions of R are n-by-n, where n is the degrees of freedom of the robot state. (n=3)

        # Q: measurement noise covariance matrix [3x3 np array]
            Note: The dimensions of Q are k-by-k, where k is the degrees of freedom of the measurement. (k=3)
    """
    def __init__(self, init_mu, init_sigma):
        self.mu = init_mu
        self.sigma = init_sigma
        self.R = np.array([[10, 0], [0, 10]])  # process noise matrix
        self.Q = ([[5, 0], [0, 5]])  # measurement noise matrix

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
            theta_t = [0,0,1];
            x_t = [1, 0, d / phi * (math.cos(theta_prev + phi) - math.cos(theta_prev))];
            y_t = [0, 1, - d / phi * (-math.sin(theta_prev + phi) + math.sin(theta_prev))];
            G = np.array([x_t, y_t, theta_t])
        return G

    @staticmethod
    def get_expected_measurement(self, pose):
        """
        h function
        Returns the expected measurement of the robot, given the robot's current state. [3-by-1 Numpy array], where
        k = 3 dimensions of measurements

        Parameters:
        # pose: robot's state
        """
        #First get data, convert data to how it relates to our pose 
        #GPS -> meters away from origin 
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

    def predict_step(self, control):
        """
        Returns the distribution of the robot's state after the prediction step of the EKF, based
        on the robot's controls.
        [mu_bar, sigma_bar]
        """
        jac_G = LocalizationEKF.get_g_jac(self.mu, control)
        mu_bar = LocalizationEKF.get_predicted_state(self.mu, control)
        sigma_bar = jac_G * self.sigma * np.transpose(jac_G) + self.R

        return mu_bar, sigma_bar

    def update_step(self, mu_bar, sigma_bar, measurement):
        """
        Returns the distribution of the robot's state after the update step of the EKF, based
        on the robot's sensor measurements.
        [mu, sigma]
        TODO: Fix this specification. update_step is a procedure and does not return anything.
        """
        jac_H = self.get_h_jac(mu_bar)
        kalman_gain = \
            (sigma_bar * np.transpose(jac_H) * inv(jac_H * sigma_bar * np.transpose(jac_H) + self.Q))

        self.mu = mu_bar + (kalman_gain @ (measurement - self.get_expected_measurement(mu_bar)))
        kh = kalman_gain * jac_H
        self.sigma = (np.eye(np.size(kh, 0)) - kh) @ sigma_bar


    # def localize(self, control, measurement):
    #     # TODO: currently, both predict and update steps are tied into one function. Should be separated
    #     mu_bar = self.prediction_func((self.mu, control))
    #     sigma_bar = self.predict_func_jacobian * self.sigma * \
    #                 np.transpose(self.predict_func_jacobian + self.process_noise)
    #
    #     kalman_gain = \
    #         (sigma_bar
    #          * np.transpose(self.measurement_func_jacobian)
    #          * inv(self.measurement_func_jacobian * sigma_bar *
    #                np.transpose(self.measurement_func_jacobian) + self.Q))
    #
    #     self.mu = mu_bar + (kalman_gain @ (measurement - self.measurement_func(mu_bar)))
    #     kh = kalman_gain * self.measurement_func_jacobian
    #     self.sigma = (np.eye(np.size(kh, 0)) - kh) @ sigma_bar

#
# class ExtendedKalmanFilter:
#     def __init__(self, initial_mu, initial_sigma, g, G, h, H, R, Q):
#         self.mu = initial_mu
#         self.sigma = initial_sigma
#         self.g = g
#         self.jac_G = G
#         self.jac_H = H
#         self.h = h
#         self.R = R
#         self.Q = Q
#         self.t = 0
#
#     def localize(self, u, z):
#         # Predict step
#         mu_bar = self.g(self.mu, u)
#         sigma_bar = self.jac_G * self.sigma * np.transpose(self.jac_G) + self.R
#
#         # Set kalman gain
#         kalman_gain = (
#                 sigma_bar
#                 * np.transpose(self.jac_H)
#                 * inv(self.jac_H * sigma_bar * np.transpose(self.jac_H) + self.Q)
#         )
#
#         # Update
#         # print("mu_bar shape: " + str(mu_bar.shape))
#         # print("kalman gain shape: "+ str(kalman_gain.shape))
#         # print("z: " + str(z))
#         # print("hmubar: " + str(self.h(mu_bar)))
#         # print("z - hmubar: " + str(z - self.h(mu_bar)))
#         self.mu = mu_bar + (kalman_gain @ (z - self.h(mu_bar)))
#         # print("mu shapee: " + str(self.mu.shape))
#         kh = kalman_gain * self.jac_H
#         self.sigma = (np.eye(np.size(kh, 0)) - kh) @ sigma_bar
#
#     def get_mu(self):
#         return self.mu
#
#     def get_sigma(self):
#         return self.sigma
#
#
# if __name__ == "__main__":
#     initial_mu = np.array([[10], [10]])
#     initial_sigma = np.array([[5, 0], [0, 5]])
#
#
#     def g_gps(pose, u):
#         return pose
#
#
#     G_gps_jac = np.array([[1, 0], [0, 1]])
#
#
#     def h_gps(pose):
#         return pose
#
#
#     H_gps_jac = np.array([[1, 0], [0, 1]])
#
#     R = np.array([[10, 0], [0, 2]])
#     Q = np.array([[2, 0], [0, 2]])
#
#     ekf_gps = ExtendedKalmanFilter(
#         initial_mu, initial_sigma, g_gps, G_gps_jac, h_gps, H_gps_jac, R, Q
#     )
#     # (m,s) = ekf_gps.get_state()
#     # print(ekf_gps.g(m,0))
#
#     fig, ax = plt.subplots()
#     ax.plot(ekf_gps.mu[0], ekf_gps.mu[1], "ro", label="Initial mu")
#
#     # e.set_clip_box(ax.bbox)
#     x_trajectory = []
#     y_trajectory = []
#     for i in range(20):
#         x_trajectory.append(ekf_gps.mu[0])
#         y_trajectory.append(ekf_gps.mu[1])
#
#         e = Ellipse(
#             (ekf_gps.mu[0], ekf_gps.mu[1]),
#             ekf_gps.sigma[0, 0],
#             ekf_gps.sigma[1, 1],
#             facecolor=None,
#             fill=False,
#             label="ellipse",
#         )
#         e.set_edgecolor("b")
#         # e.set_facecolor(None)
#         ax.add_artist(e)
#         ekf_gps.localize(0, np.array([[5], [6]]))
#     ax.plot(x_trajectory, y_trajectory, "g-", label="EKF trajectory")
#
#     ax.set_title("Simple EKF Localization using GPS-like Data")
#     ax.set_xlim(0, 20)
#     ax.set_ylim(0, 20)
#     ax.legend()
#     plt.show()
