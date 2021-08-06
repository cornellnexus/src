import unittest
import numpy as np
import math
from engine.astar import Astar, Node

'''
Unit tests for astar.py
'''

class TestAstar(unittest.TestCase):

  test_nodes = [
    Node(0,0,1000,3), Node(0,1,1000,3), Node(0,2,1000,3),
    Node(1,0,1000,3), Node(1,1,1000,3), Node(1,2,1000,3),
    Node(2,0,1000,3), Node(2,1,1000,3), Node(2,2,1000,3)

    ]


# Tests for node class in astar.py
  def test_node_get_pos(self):
    ans_set = [
      (0,0), (0,1), (0,2),
      (1,0), (1,1), (1,2),
      (2,0), (2,1), (2,2),
      ]
    for (ans, node) in zip(ans_set, self.test_nodes):
        self.assertEqual(ans, node.get_pos())

  def test_make_grid(self):
    pass
  
  def test_heuristic(self):
    pass

  def test_algorithm(self):
    pass


if __name__ == '__main__':
    unittest.main()