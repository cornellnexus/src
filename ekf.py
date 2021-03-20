import numpy as np
from numpy.linalg import inv

class ExtendedKalmanFilter:
    def __init__(self, initial_mu, initial_sigma, g, h, R, Q):
        self.mu = initial_mu
        self.sigma = initial_sigma
        self.g = g
        self.h = h
        self.R = R
        self.Q = Q
        self.t = 0

    def localize(self, u, z):
        # Predict step
        mu_bar = self.g(self.mu, u)
        sigma_bar = self.G * self.sigma * np.transpose(self.G) + R

        # Set kalman gain
        kalman_gain = sigma_bar * self.H * inv(self.H * sigma_bar * \
            np.transpose(self.H) + self.Q)

        #Update
        self.mu = mu_bar + kalman_gain(z - h(mu_bar))
        self.sigma = (I - (kalman_gain * self.H)) * sigma_bar

    def get_state(self):
        return (self.mu, self.sigma)