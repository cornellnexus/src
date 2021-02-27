 from collections import deque

import numpy as np
import random
import time 
import math
import matplotlib.pyplot as plt
import pid_controller

class Robot:
    def __init__(self, position= [0,0], heading = 90): 
        self.pos = position
        self.heading = heading   
    
    def print_current_state(self):
        print('pos: ' + str(self.pos))
        print('heading: ' + str(self.heading))
    
    def move_forward(self,v,t,w=0.0):
        if self.heading == 90:
            self.pos[1] = self.pos[1] + v * t
        elif self.heading == 270: 
            self.pos[1] = self.pos[1] - v * t
        elif self.heading == 0: 
            self.pos[0] = self.pos[0] + v * t
        self.print_current_state()

    def turn_left(self, w, t):
        self.heading += (w * t) % 360
        self.print_current_state()
    
    def turn_right(self):
        self.heading -= (w * t) % 360
        self.print_current_state()
    
    def get_position(self):
        return self.pos

# TODO: To be used in final test.
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


if __name__ == "__main__":
    #longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()
    # TEST1- for testing purposes, initialize pids as follows:
    loc_pid = PID(Kp = 0.01, Ki = 0.0, Kd = 0.0, target = 0, sample_time = 1.0, output_limits = (None, None))
    head_pid = PID(Kp = 0.01, Ki = 0.0, Kd = 0.0, target = 0, sample_time = 1.0, output_limits = (None, None))
    # g = generate_nodes()
    # TEST1 - Use one node for the queue
    g = [(0,10)]
    queue = deque(g)
    r = Robot()

    while queue:
        target_coords = queue.popleft()  # Next node to visit from grid
        predicted_loc = r.get_position() # Assuming prediciton is 100% accurate
        predicted_head = r.get_heading() # Assuming prediciton is 100% accurate

        #distance formula 
        def get_distance(x_targ,x_pred, y_targ,y_pred):
            return math.sqrt((x_targ-x_pred)**2 + (y_targ-y_pred)**2)
       
        # # TODO: confirm location error = pythagorean or 2d? 
        # TEST1 - location_error should be initially 10.
        location_error = \
            get_distance(target_coords[0],predicted_loc[0],\
            target_coords[1],predicted_loc[1])
        allowed_error = 0 #TODO: measure this

        # while robot is too far away from target node
        while location_error > allowed_error:
            loc_pid = loc_pid.set_target(target_coords)  
            head_pid = head_pid.set_target(0) 
            vel = loc_pid.update(predicted_loc) 
            ang_vel = head_pid.update(predicted_head) #TEST1 - should be 0 
            
            # TODO: send angular_velocity and velocity to electrical
            r.move_forward(vel,loc_pid.get_sample_time())
            # The robot moves forward adjusting to move in straight line. 
            sleep(loc_pid.get_sample_time())
            
            #TEST1 - Get current location from fake robot (IRL Kalman Filter)
                predicted_loc = r.get_position()
            location_error = \
                get_distance(target_coords[0],predicted_loc[0],\
                target_coords[1],predicted_loc[1])  

        # We have reached the target node. Sanity check for stopping:
        r.move_forward(0,0)
        # Not necessary for dummy test:
        # sleep(robot_stop_time)

        # TEST 1 - Not necessary to adjust heading
        # Turning Left and Right 
        # if target_coords[1] == 9: 
        #     print("Turning right")
        #     next_target_coords = queue.peek()
        #     if target_coords[0] < next_target_coords[0]:
        #         target_angle = 0
        #     elif target_coords[0] == next_target_coords[0]:
        #         target_angle = 270
        #     print("Should not be here, faulty angle logic")
        # elif target_coords[0] != 0 and target_coords[1] == 0:
        #     print("Turning left")
        #     next_target_coords = queue.peek()
        #     if target_coords[0] < next_target_coords[0]:
        #         target_angle = 0
        #     elif target_coords[0] == next_target_coords[0]:
        #         target_angle = 90

        # angle_error = abs(target_angle - r.heading)
        # allowed_error = 2   # TODO: Measure this   
        # while angle_error > allowed_error:
        #     head_pid.set_target(target_angle)
        #     angular_velocity = head_pid.update(angle_error)
        #     # TODO: send angular_velocity to electrical and (velocity = 0)
        #     # The robot turns.
        #     sleep(head_pid.get_sample_time())
        #     # TODO: Get current heading from Kalman filter
        #     # r.set_heading = Kalman filter output
        #     angle_error = abs(target_angle - r.heading)
    print("Reached end of traversal path!")

# -----------------------------PLOT PATH----------------------------------------
# we should make this a function 
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

    # for node in history:
    #     print("Plotting node")
    #     plt.plot(node[0],node[1],marker='x',markerfacecolor='red')
    #     time.sleep(.1)
    #     plt.show()
    # plt.close()

# if __name__ == "__main__":