from node import *
import unittest

'''
Unit tests for node.py
'''

class TestNodes(unittest.TestCase):
    #Set up nodes to test on
    reg_node = Node(20, 30)

    border_node = Node(20, 30, '1')
    non_border_node = Node(20, 30, '0')

    non_traversed_node = Node(20, 30, status =0)
    traversed_node = Node(20, 30, status = 1)
    obstacle_node = Node(20, 30, status =2)

    non_trav_border_node = Node(20, 30, '1',status =0)
    trav_border_node =  Node(20, 30, '1',status = 1)
    obstacle_border_node = Node(20, 30, '1',status =2)

    non_trav_non_border_node = Node(20, 30, '0',status = 0)
    trav_non_border_node =  Node(20, 30, '0',status = 1)
    obstacle_non_border_node = Node(20, 30, '0',status =2)

    max_lat_node = Node(90, 30)
    min_lat_node = Node(-90, 30)
    max_long_node = Node(20, 180)
    min_long_node = Node(20, -180)

    #Group initialized nodes together
    test_set = [
        reg_node, border_node, non_border_node, non_traversed_node,
        traversed_node, obstacle_node, non_trav_border_node, trav_border_node,
        obstacle_border_node, non_trav_non_border_node, trav_non_border_node,
        obstacle_non_border_node, max_lat_node, min_lat_node, max_long_node, 
        min_long_node
        ]

    #Test/edge cases that should fail when initialized?
    neg_lat_node = Node(-100, 30)
    pos_lat_node = Node(150, 30)
    neg_long_node = Node(20, -200)
    pos_long_node = Node(20, 200)

    invalid_pos_status_node = Node(20, 30, status = 5)
    invalid_neg_status_node = Node(20, 30, status = -1)

    invalid_is_border_node = Node(20, 30, is_border = '3')



    def test_get_coords(self):
        self.assertEqual((20,30), self.reg_node.get_coords())

    def test_is_border_node(self):
        #'0' == False, '1' == True? is_border_node should return boolean?
        self.assertEqual('0', self.reg_node.is_border_node())
        self.assertEqual('1', self.border_node.is_border_node())

    def test_get_status(self):
        ans_set = [0,0,0,0,1,2,0,1,2,0,1,2,0,0,0,0]
        for (a, t) in zip(ans_set, self.test_set):
            self.assertEqual(a, t.get_status())

    def test_set_status(self):
        #increment status by 1, but if 2, set to 0
        for node in self.test_set:
            status = node.status
            if status == 2:
                node.set_status(0)
            else:
                node.set_status(status+1)

        ans_set = [1,1,1,1,2,0,1,2,0,1,2,0,1,1,1,1]

        for (a, t) in zip(ans_set, self.test_set):
                    self.assertEqual(a, t.get_status())

    def test_eq(self):
        self.assertEqual(True, self.reg_node == self.reg_node)
        self.assertEqual(True, self.max_lat_node == Node(90,30))
        self.assertEqual(False, self.non_traversed_node == self.obstacle_border_node)
        self.assertEqual(False, self.max_lat_node == self.min_lat_node)
        #Nodes with different is_border values should not be equal?
        # self.assertEqual(False, self.border_node == self.non_border_node)

    def test_repr(self):
        self.assertEqual("(20,30)", repr(self.reg_node))

if __name__ == '__main__':
    unittest.main()
