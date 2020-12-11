# from UserUtils import *
# from FileUtils import *
# # from Graph import Graph
# from GUI import GUI
# from obstacle import *
# from Graph import *

import csv


    # def run(self):
        
    #     coordsList = graph.startTraversal()

    # def obstacle_path(self):
        
    #     queue = deque(self.traversalPath[:]) #make a copy of traversal list
    #     closed = []
    #     current_vertex = Coordinate(self.robot_starting_position[0],
    #                                 self.robot_starting_position[1])

    #     while queue:
    #         vertex = queue.popleft()  #Next node to visit

    #         if vertex.getProbability() == 1: # if closed, then skip it
    #             continue

    #         self.check_for_obstacle(vertex)
    #         if vertex.getProbability() == 2:
    #             branched = encircle_obstacle(current_vertex, queue)
    #             obstacle_coordinates += branched
    #         (x,y) = vertex.getCoords()

    #         closed.append(vertex)
    #         self.moveRobotToCoordinate(vertex, current_vertex)
    #         #stop function

    #         current_vertex = vertex # succssfully moed to desired node
    #     return rawCoordsTraversed


if __name__ == "__main__":
    longMin, longMax, latMin, latMax = getLongLatMinMaxFromUser()

    # Create graph object given longitute and latitude coordinates from user input.
    graph = Graph(longMin, longMax, latMin, latMax)
    queue = deque(list.copy.deepcopy(graph.traversalPath))

    while queue:
        next_node = queue.popleft() #Next node to visit
        if next_node.get_status() == 1: # if closed, then skip it
            continue
        # check for obstacle 
        elif next_node.get_status() == 0:  
            predicted_loc = update_step()
            

            #math.dist([Px, Py], [Qx, Qy]) 
            #distance_from_target = math.dist(predicted_loc, next_node.get_coords)
            
            #distance_from_target <- get pythogerean distance from target in meters
            distance_from_target = geopy.distance.distance(predicted_loc,next_node.get_coords).meters
            gps_noise_range = 3
            while distance_from_target <  gps_noise_range:
                self.ser.write(b'F')
                predicted_loc = update_step()
                distance_from_target = geopy.distance.distance(predicted_loc,next_node.get_coords).meters
            # Add support for turning L and R.

    # with open('longandlats.csv', newline='') as csvfile:
    #     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        
    #     for row in spamreader:
    #         print(', '.join(row))
            
 






# engine = Engine()
# engine.run()
