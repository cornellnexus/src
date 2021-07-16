import unittest
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation as animation
from matplotlib import patches as patch
import time
import random

import os.path
cwd = os.getcwd()
import sys
sys.path.append(cwd[0:cwd.index('compilation_tests')-1])
from sim_trajectory import *
from pid_controller import PID
from robot import Robot
from kinematics import limit_cmds, feedback_lin, integrate_odom
from UserUtils import *
from node import *

target = [0,0]
add_to_x = False
gps_noise_range = .3
# lat_min, lat_max, long_min, long_max = get_coord_inputs()
# current_position = [lat_min, long_min]

class TestGenerateNodes(unittest.TestCase):
    def test_no_noise_all_nodes(self):
      r2d2 = Robot(-5,-10,math.pi/2)
      NOISE_RANGE = 0.0

      goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
      [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
      [10,0],[10,-5],[10,-10]])


      Kp=1
      Ki=0.1
      Kd=0.1

      all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
      self.assertEqual([True] * len(goals.tolist()), all_nodes_traveled_to)

    def test_no_noise_first_target(self):
      r2d2 = Robot(-5, -10, math.pi/2)
      NOISE_RANGE = 0.0

      goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
      [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
      [10,0],[10,-5],[10,-10]])

      Kp=1
      Ki=0.1
      Kd=0.1

      all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
      self.assertEqual([True] * 2, all_nodes_traveled_to[0:2])

    def test_no_noise_first_turn(self):
      r2d2 = Robot(-5, -10, math.pi/2)
      NOISE_RANGE = 0.0

      goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
      [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
      [10,0],[10,-5],[10,-10]])

      Kp=1
      Ki=0.1
      Kd=0.1

      all_nodes_traveled_to = simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
      self.assertEqual([True] * 3, all_nodes_traveled_to[0:3])

    # def test_minimal_noise(self):
    #   r2d2 = Robot(-5,-10,math.pi/2)
    #   NOISE_RANGE = 0.1

    #   goals = np.array([[-5,-10],[-5,-5],[-5,0],[-5,5],[-5,10],[0,10],[0,5],\
    #   [0,0],[0,-5],[0,-10],[5,-10],[5,-5],[5,0],[5,5],[5,10],[10,10],[10,5],\
    #   [10,0],[10,-5],[10,-10]])

    #   Kp=1
    #   Ki=0.1
    #   Kd=0.1

    #   simulation(r2d2, NOISE_RANGE, goals, Kp, Ki, Kd)
    
# class TestPlottingFunctions(unittest.TestCase):
#   def test_waypoints_to_array(self):
#     #Create node_list and corresponding numpy array
#     node_list = [] 
#     ans_node_list =[]
#     for i in range(0, 22):
#       for j in range(0, 10):
#         node_list.append(Node(i,j))
#         ans_node_list.append([float(i),float(j)])
#     ans_node_list = np.array(ans_node_list)

#     #Check equality
#     result = np.array_equal(ans_node_list, waypoints_to_array(node_list))
#     self.assertEqual(True, result)
    


if __name__ == '__main__':
    unittest.main()