import unittest
from node import Node
from grid import Grid
import matplotlib.pyplot as plt

def graph_traversal_path(traversal_path):
    print(traversal_path)
    xlist = []
    ylist = []

    for node in traversal_path:
        coords = node.get_coords()
        xlist.append(coords[1])
        ylist.append(coords[0])

    plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
    plt.ylim(min(ylist) - 0.00001,max(ylist) + 0.00001)
    plt.xlim(min(xlist) - 0.00001,max(xlist) + 0.00001)
    plt.show()
    plt.clf()
    plt.cla()
    plt.close()

class TestGenerateNodes(unittest.TestCase):
    def test_jessica_house(self):
        g = Grid(34.23117305494089, 34.23120742746021, -119.00640453979496, -119.00629523978851)
        graph_traversal_path(g.traversal_path)

    def test_pike_room(self):
        g = Grid(-76.488495, -76.488419, 42.444496, 42.444543)
        graph_traversal_path(g.traversal_path)

if __name__ == '__main__':
    unittest.main()


################################################################################
# Below is cayuga plot
# g = Graph(42.4596, 42.4642, -76.5119, -76.5013)

# testlist = []
# xlist = []
# ylist = []

# for node in g.traversal_path:
#     coords = node.get_coords()
#     xlist.append(coords[0])
#     ylist.append(coords[1])

# plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
# plt.ylim(-76.5149, -76.5000)
# plt.xlim(42.4580, 42.4650)# g = Graph(42.444496,42.444543, -76.488495, -76.488419)
