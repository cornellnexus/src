from collections import deque
from pid_controller import PID
import numpy as np
import random
import time 
import math
import matplotlib.pyplot as plt

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
    
    def turn_right(self, w, t):
        self.heading -= (w * t) % 360
        self.print_current_state()
    
    def get_position(self):
        return self.pos

    def get_heading(self):
        return self.heading

def engine_pid_dummy():
    #longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()
    # TEST1- for testing purposes, initialize pids as follows:
    loc_pid = PID(Kp = 0.1, Ki = 0.0, Kd = 0.0, target = 0, sample_time = 1.0, output_limits = (None, None))
    head_pid = PID(Kp = 0.1, Ki = 0.0, Kd = 0.0, target = 0, sample_time = 1.0, output_limits = (None, None))

    loc_pid.am_i_real()
    # g = generate_nodes()
    # TEST1 - Use one node for the queue
    # TEST 1: g = [(0,5)]
    # TEST 2: move forward and turn right 
    g = [(0,5), (5,5)]
    queue = deque(g)
    r = Robot()

    while queue:
        target_coords = queue.popleft()  # Next node to visit from grid
        print("----------TARGET NODE: " + str(target_coords) + "---------------") 
        predicted_loc = r.get_position() # Assuming prediciton is 100% accurate
        print("----------PREDICTED LOC: " + str(predicted_loc) + "---------------")
        predicted_head = r.get_heading() # Assuming prediciton is 100% accurate

        #distance formula 
        # TODO: fix with 2d error not pythagorean
        def get_distance(x_targ,x_pred, y_targ,y_pred):
            return math.sqrt((x_targ-x_pred)**2 + (y_targ-y_pred)**2)
       
        # # TODO: confirm location error = pythagorean or 2d? 
        # TEST1 - location_error should be initially 10.
        location_error = \
            get_distance(target_coords[0],predicted_loc[0],\
            target_coords[1],predicted_loc[1])
        allowed_error = 1 #TODO: measure this

        # while robot is too far away from target node
        while location_error > allowed_error:
            loc_pid.set_target(target_coords)  
            head_pid.set_target(0) 
            #TODO: Calling update wrong - if confirmed, also fix in engine_pid 
            print("LOCATION ERROR: " + str(location_error))
            vel = loc_pid.update(location_error)
            print("LOC PROP: "+ str(loc_pid.get_proportional()))
            print("LOC DERIV: "+ str(loc_pid.get_derivative()))
            print("LOC INTEG: "+str(loc_pid.get_integral()))
            print("VEL: "+ str(vel))
            ang_vel = head_pid.update(0) #TEST1 - should be 0 
            print("HEAD PROP: "+ str(head_pid.get_proportional()))
            print("HEAD DERIV: "+ str(head_pid.get_derivative()))
            print("HEAD INTEG: "+ str(head_pid.get_integral()))
            print("ANG VEL: "+ str(ang_vel))
            # TODO: send angular_velocity and velocity to electrical
            r.move_forward(vel,loc_pid.get_sample_time())
            # The robot moves forward adjusting to move in straight line. 
            time.sleep(loc_pid.get_sample_time())
            
            #TEST1 - Get current location from fake robot (IRL Kalman Filter)
            predicted_loc = r.get_position()
            location_error = \
                get_distance(target_coords[0],predicted_loc[0],\
                target_coords[1],predicted_loc[1])  

        # We have reached the target node. Sanity check for stopping:
        print("------------------ REACHED TARGET NODE ------------------")
        r.move_forward(0,0)
        # Not necessary for dummy test:
        # sleep(robot_stop_time)

        # TEST 1 - Not necessary to adjust heading
        # Turning Left and Right 
        if target_coords[1] == 5: 
            print("Turning right")
            # Check if 
            next_target_coords = queue[0] # peek
            if target_coords[0] < next_target_coords[0]:
                target_angle = 0
            elif target_coords[0] == next_target_coords[0]:
                target_angle = 270
            print("Should not be here, faulty angle logic")
        elif target_coords[0] != 0 and target_coords[1] == 0:
            print("Turning left")
            next_target_coords = queue[0] # peek
            if target_coords[0] < next_target_coords[0]:
                target_angle = 0
            elif target_coords[0] == next_target_coords[0]:
                target_angle = 90

        angle_error = abs(target_angle - r.heading)
        allowed_error = 2   # TODO: Measure this   
        while angle_error > allowed_error:
            head_pid.set_target(target_angle)
            angular_velocity = head_pid.update(angle_error)
            print("HEAD PROP: "+ str(head_pid.get_proportional()))
            print("HEAD DERIV: "+ str(head_pid.get_derivative()))
            print("HEAD INTEG: "+ str(head_pid.get_integral()))
            print("ANG VEL: "+ str(angular_velocity))
            # TODO: send angular_velocity to electrical and (velocity = 0)
            # DUMMY TEST 2: ONLY SUPPORTS TURNING RIGHT!!!
            r.turn_right(angular_velocity, head_pid.get_sample_time())
            # The robot turns.
            time.sleep(head_pid.get_sample_time())
            # TODO: Get current heading from Kalman filter
            # # r.heading = Kalman filter output
            angle_error = abs(target_angle - r.get_heading())
        print("----------------- DONE WITH TURNING ROUTINE -----------------")
    print("Reached end of traversal path!")


# TEST 2 fails to get to end node because turning and moving logic requires 
# exact angles to be met. If we allow for angle error, the robot will not keep
# moving. Consider changing turning logic to a more mechanically intuitive thing?

if __name__ == "__main__":
    engine_pid_dummy()


# -----------------------------PLOT PATH----------------------------------------
# we should make this a function 
    # xlist = []
    # ylist = []
    # for node in g:
    #     xlist.append(node[0])
    #     ylist.append(node[1])
    # plt.plot(xlist, ylist, 'ro',markerfacecolor='blue')
    # plt.ylim(min(ylist) - 1,max(ylist) + 1)
    # plt.xlim(min(xlist) - 1,max(xlist) + 1)
    # plt.show()

    # xlist2=[]
    # ylist2=[]
    # plt.plot(xlist2, ylist2, 'bx')
    # plt.ylim(min(ylist2) - 1,max(ylist2) + 1)
    # plt.xlim(min(xlist2) - 1,max(xlist2) + 1)
    # plt.show()
    # plt.close()
#TODO: Add plotting function that plots robot trajectory against grid Nodes!
