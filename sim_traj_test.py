import unittest
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
from sim_trajectory import simulation

target = [0,0]
add_to_x = False
gps_noise_range = .3
max_lat = 5
max_long = 5
current_position = [0,0]

class TestGenerateNodes(unittest.TestCase):
    def test_no_noise(self):
      r2d2 = Robot(-5,-10,math.pi/2)
      NOISE_RANGE = 0.0

      goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
      [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
      [10,0],[10,-5],[10,-10]])

      Kp=1
      Ki=0.1
      Kd=0.1

      simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)


    def test_minimal_noise(self):
      r2d2 = Robot(-5,-10,math.pi/2)
      NOISE_RANGE = 0.1

      goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
      [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
      [10,0],[10,-5],[10,-10]])

      Kp=1
      Ki=0.1
      Kd=0.1

      simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)


if __name__ == '__main__':
    unittest.main()