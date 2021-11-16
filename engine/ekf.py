import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


class LocalizationEKF:
    """
    An implementation of the EKF localization for the robot.
    """
    def __init__(self, init_mu, init_sigma):
        self.mu = init_mu
        self.sigma = init_sigma
        self.predict_func_jacobian = np.array([[1, 0], [0, 1]])
        self.measurement_func_jacobian = np.array([[1, 0], [0, 1]])
        self.process_noise = np.array([[10, 0], [0, 10]])
        self.measurement_noise([[5, 0], [0, 5]])

    @staticmethod
    def prediction_func(self, pose, control):
        return pose

    @staticmethod
    def measurement_func(self, pose):
        return pose

    def localize(self, control, measurement):
        # TODO: currently, both predict and update steps are tied into one function. Should be separated
        mu_bar = self.prediction_func((self.mu, control))
        sigma_bar = self.predict_func_jacobian * self.sigma * \
                    np.transpose(self.predict_func_jacobian + self.process_noise)

        kalman_gain = \
            (sigma_bar
             * np.transpose(self.measurement_func_jacobian)
             * inv(self.measurement_func_jacobian * sigma_bar *
                   np.transpose(self.measurement_func_jacobian) + self.Q))

        self.mu = mu_bar + (kalman_gain @ (measurement - self.measurement_func(mu_bar)))
        kh = kalman_gain * self.measurement_func_jacobian
        self.sigma = (np.eye(np.size(kh, 0)) - kh) @ sigma_bar

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
