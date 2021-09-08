import unittest
import matplotlib.pyplot as plt

from engine.node import Node
from engine.grid import Grid


def graph_traversal_path(g, map_name, distance_type, mode):
    """
    Plots both the gps and meters grid generated by Grid.py
    """
    gps_traversal_path = g.gps_waypoints
    meters_traversal_path = g.get_waypoints(mode)
    print('----------METERS TRAVERSAL PATH--------')
    print(meters_traversal_path)
    gps_xlist = []
    gps_ylist = []
    m_xlist = []
    m_ylist = []

    for node in gps_traversal_path:
        coords = node.get_coords()
        gps_xlist.append(coords[1])
        gps_ylist.append(coords[0])

    for node in meters_traversal_path:
        coords = node.get_coords()
        m_xlist.append(coords[0])
        m_ylist.append(coords[1])

    plot1 = plt.figure(1)
    plt.plot(gps_xlist, gps_ylist, marker='o', markerfacecolor='blue')
    plt.ylim(min(gps_ylist) - 0.000001, max(gps_ylist) + 0.000001)
    plt.xlim(min(gps_xlist) - 0.000001, max(gps_xlist) + 0.000001)
    plt.title('Grid in GPS coordinates')
    # for i_x, i_y in zip(gps_xlist, gps_ylist):
    #     plt.text(i_x, i_y, '({}, {})'.format(i_x, i_y))

    plot2 = plt.figure(2)
    plt.plot(m_xlist, m_ylist, marker='o', markerfacecolor='blue')
    plt.ylim(min(m_ylist) - 1, max(m_ylist) + 1)
    plt.xlim(min(m_xlist) - 1, max(m_xlist) + 1)
    # for i_x, i_y in zip(m_xlist, m_ylist):
    #     plt.text(i_x, i_y, '({}, {})'.format(i_x, i_y))
    plt.title(map_name + ' Grid Converted to Meters ' + '(' + distance_type + ')')
    plt.show()

    plt.close()


class TestGenerateNodes(unittest.TestCase):
    # def test_jessica_house(self):
    # # g = Grid(34.23117305494089, 34.23120742746021, -119.00640453979496, -119.00629523978851)
    # g = Grid(-76.483682, -76.483276, 42.444250, 42.444599)
    # graph_traversal_path(g.gps_traversal_path)

    # def test_pike_room(self):
    #     g = Grid(-76.488495, -76.488419, 42.444496, 42.444543)
    #     graph_traversal_path(g, 'Pike Room', 'Vincenty')

    def test_engineering_quad(self):
        g = Grid(42.444250, 42.444599, -76.483682, -76.483276)
        # g = Grid(42.444000, 42.444600, -76.483600, -76.483000)
        graph_traversal_path(g, 'Engineering Quad', 'Vincenty', 'full')

    def test_border_nodes(self):
        count = 0
        g = Grid(42.444250, 42.444599, -76.483682, -76.483276)
        full_waypoints = g.get_waypoints('full')
        for nd in full_waypoints:
            if nd.is_border_node(): count += 1
        self.assertNotEqual(count, g.get_num_rows() * g.get_num_cols())
        self.assertEqual(count, g.get_num_cols() * 2)


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