from collections import deque
import numpy as np
import random
import time 
import math
import matplotlib.pyplot as plt

"""
Defines a dummy robot to be used for debugging
"""
class Robot:
    """
    Initializes the robot with the given position and heading.
    Parameters:
    position: int list representing x and y position of robot
    heading: angle robot is facing from East. (E.g. 0 East, 90 North, 270 South)
    Preconditions:
    position is list of size 2
    heading is in range [0..359]
    """
    def __init__(self, position= [0,0], heading = 90): 
        self.pos = position
        self.heading = heading
    """
    Returns randomly generated noise in range [-.02, .02]
    """
    def get_noise(self):
        return np.random.uniform(-.02, .02)
    """
    Prints robots current position and heading
    """
    def print_current_state(self):
        print('pos: ' + str(self.pos))
        print('heading: ' + str(self.heading))
        # time.sleep(0.1)    
    """
    Moves robot forward by an increment of 0.1
    If facing North, increment y coordinate.
    If facing South, decrement y coordinate.
    If facing East, increment x coordinate.
    For the purposes of this test, Robot does not ever face West.
    Warning: This method only works assuming the only possible headings are 
    90, 270, or 0.
    """
    def move_forward(self): 
        if self.heading == 90:
            self.pos[1] = round(self.pos[1] + .1,3)
            # self.pos[1] = round(self.pos[1] + .1 + self.get_noise(),3)
        elif self.heading == 270: 
            self.pos[1] = round(self.pos[1] - .1,3)
            # self.pos[1] = round(self.pos[1] - .1 + self.get_noise(),3)
        elif self.heading == 0: 
            self.pos[0] = round(self.pos[0] + .1,3)
            # self.pos[0] = round(self.pos[0] + .1 + self.get_noise(),3)
        self.print_current_state()
    """
    Updates heading to turn robot left 90 degrees.
    """
    def turn_left(self):
        self.heading += 90
        self.heading = self.heading % 360
        self.print_current_state()  
    """
    Updates heading to turn robot right 90 degrees.
    """
    def turn_right(self):
        self.heading -= 90
        self.heading = self.heading % 360
        self.print_current_state()
    """
    Current position of robot
    """
    def get_position(self):
        return self.pos

# Create graph object given longitute
# and latitude coordinates from user input.
# g = Grid(longMin, longMax, latMin, latMax)
# Create a grid from 0..10 and 0..10
def generate_nodes():
    origin = (0, 0)
    traversal_path = []
    for i in range(10):
        for j in range(10):
            if i % 2 == 0:
                node = origin[0] + i, origin[1] + j
                # node = Node(origin[0] + j, origin[1] + i)
                traversal_path.append(node)
            elif i % 2 == 1:
                # lat_pos = origin[0] + (9 - j)
                # long_pos = origin[1] + i
                y = origin[1] + (9-j)
                x = origin[0] + i 
                # node = Node(lat_pos, long_pos)
                node = (x,y)
                traversal_path.append(node)
    return traversal_path


"""
Plots traversal path.
WARNING: This method is not used, should we remove it?
"""
def graph_traversal_path(traversal_path):
    print(traversal_path)
    xlist = []
    ylist = []

    for node in traversal_path:
        xlist.append(node[0])
        ylist.append(node[1])

    plt.plot(xlist, ylist, marker='o', markerfacecolor='blue')
    plt.ylim(min(ylist) - 1,max(ylist) + 1)
    plt.xlim(min(xlist) - 1,max(xlist) + 1)
    plt.show()
    plt.close()
    # plt.clf()
    # plt.cla()
    # plt.close()


"""
Moves dummy robot along the traversal path and plots the traversal.
"""
def engine_dummy():
    #longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()
    g = generate_nodes()
    queue = deque(g)
    r = Robot()
    print("Beginning movement")
    history = []
    while queue:
        print("Selecting new nodes")
        target_coords = queue.popleft()  # Next node to visit from grid
        
        # returns GPS data in the form (lat,long)
        #predicted_loc = update_step() # robot
        predicted_loc = r.get_position()

        # must be in form latitude,longitude.
        # distance_from_target = 1
        #distance formula 
        def get_distance(x_targ,x_pred, y_targ,y_pred):
            return math.sqrt((x_targ-x_pred)**2 + (y_targ-y_pred)**2)
        # distance should be close to 1 
        distance_from_target = \
            get_distance(target_coords[0],predicted_loc[0],\
            target_coords[1],predicted_loc[1])
        gps_noise_range = 0
        # gps_noise_range = 0.13

        # while robot is too far away from target node
        while distance_from_target > gps_noise_range:
            # move forward command; talk to electrical about moving
            r.move_forward()
            print("Target node: x is " + str(target_coords[0]) + ", y is " + str(target_coords[1]))
            # Get predicted location from robot 
            predicted_loc = r.get_position()
            history.append(predicted_loc[:])
            distance_from_target = \
                get_distance(target_coords[0],predicted_loc[0],\
                target_coords[1],predicted_loc[1])  
            print("distance from target is now" + str(distance_from_target))
            print("-----------------------------------")


        # We are currently at target node (next_node)
        print("\n\nReached target node: " + str(target_coords)+ "\n\n")
        print("Actual location at target node: " + \
            str(r.get_position()[0])+","+str(r.get_position()[1]))
        # Add support for turning L and R
        if target_coords[1] == 9: 
            print("Turning right")
            r.turn_right()
        elif target_coords[0] != 0 and target_coords[1] == 0:
            print("Turning left")
            r.turn_left()
    print("Reached end of traversal path!")

    # Plotting traversal path:
    xlist = []
    ylist = []
    for node in g:
        xlist.append(node[0])
        ylist.append(node[1])
    plt.plot(xlist, ylist, 'ro',markerfacecolor='blue')
    plt.ylim(min(ylist) - 1,max(ylist) + 1)
    plt.xlim(min(xlist) - 1,max(xlist) + 1)
    plt.show()

    xlist2=[]
    ylist2=[]
    for node in history:
        xlist2.append(node[0])
        ylist2.append(node[1])
    plt.plot(xlist2, ylist2, 'bx')
    plt.ylim(min(ylist2) - 1,max(ylist2) + 1)
    plt.xlim(min(xlist2) - 1,max(xlist2) + 1)
    plt.show()
    plt.close()


if __name__ == "__main__":
    engine_dummy()

    # for node in history:
    #     print("Plotting node")
    #     plt.plot(node[0],node[1],marker='x',markerfacecolor='red')
    #     time.sleep(.1)
    #     plt.show()
    # plt.close()