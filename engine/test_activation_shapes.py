import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

class Node:
    def __init__(self,activated,x,y):
        self.activated = activated
        self.on_border = False
        self.x = x
        self.y = y


def activate_rectangle(nodes, row, col, row_limit, col_limit):
        """
        Activates all the nodes in a rectangle.
        """
        for x in range(row, row_limit):
            for y in range(col, col_limit):
                nodes[x,y].activated = True
        return nodes

def activate_circle(nodes,circle_center_row, circle_center_col, circle_radius):
    """
    Activates all the nodes in a circle.
    """
    rows = nodes.shape[0]
    cols = nodes.shape[0]
    for x in range(rows):
        for y in range(cols):
            if ((x-circle_center_row)**2 + (y-circle_center_col)**2 - circle_radius**2) < 0:
                nodes[x,y].activated = True
    return nodes

def area(x1, y1, x2, y2, x3, y3):
 
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)
 
 
# A function to check whether point P(x, y)
# lies inside the triangle formed by
# A(x1, y1), B(x2, y2) and C(x3, y3)
def isInsideTriangle(x1, y1, x2, y2, x3, y3, x, y):
 
    # Calculate area of triangle ABC
    A = area (x1, y1, x2, y2, x3, y3)
 
    # Calculate area of triangle PBC
    A1 = area (x, y, x2, y2, x3, y3)
     
    # Calculate area of triangle PAC
    A2 = area (x1, y1, x, y, x3, y3)
     
    # Calculate area of triangle PAB
    A3 = area (x1, y1, x2, y2, x, y)
     
    # Check if sum of A1, A2 and A3
    # is same as A
    if(A == A1 + A2 + A3):
        return True
    else:
        return False

def activate_traingle(nodes,x1,y1,x2,y2,x3,y3):
    """
    Activates all the nodes in a traingle.
    """
    rows = nodes.shape[0]
    cols = nodes.shape[0]
    for x in range(rows):
        for y in range(cols):
            if isInsideTriangle(x1,y1,x2,y2,x3,y3,x,y):
                nodes[x,y].activated = True
    return nodes

def is_on_border(nodes,row, col, row_limit, col_limit):
        min_col = max(0, col-1)
        min_row = max(0, row-1)
        max_col = min(col_limit, col+1)
        max_row = min(row_limit, row+1)

        # If this node is on the very edge of the grid, it is automatically a border node
        if min_col == 0 or min_row == 0 or max_col == col_limit or max_row == row_limit:
            return True

        # If the node has a neighboring node that is inactive, it is a border node
        for col in range(min_col, max_col+1):
            for row in range(min_row, max_row+1):
                if not nodes[row][col].activated:
                    return True
        return False


def find_border_nodes(nodes):
    border_list = []
    cols = nodes.shape[1]
    rows = nodes.shape[0]
    leftmost_node = None
    leftmost_node_pos = None
    for row in range(rows):
        for col in range(cols):
            node = nodes[row][col]
            if node.activated and is_on_border(nodes,row, col, rows-1, cols-1):
                # check if this is an active node and on the border
                nodes[row][col].on_border = True
                border_list.append(node)
                if leftmost_node_pos is None or col < leftmost_node_pos[1]:
                    leftmost_node = node
                    leftmost_node_pos = (row, col)
    border_nodes = border_list
    leftmost_node = leftmost_node
    leftmost_node_pos = leftmost_node_pos
    return border_nodes,leftmost_node,leftmost_node_pos, nodes

##Return bottom most node that is activated in the right column
def return_right_position(pos, border_nodes):
    candidate_nodes = [node for node in border_nodes if node.y == pos[1]+1]
    if (candidate_nodes == []):
        return None
    else:
        node = min(candidate_nodes,key=lambda node: node.x)
        return (node.x,node.y)

##Computes the lawnmover points for the grid using the activated and border nodes
def get_all_lawnmower_waypoints_adjustable(nodes,lefmost_node_pos,border_nodes):
        class WaypointPhase(Enum):
            DOWN = 1
            TERMINATE = 2
        rows = nodes.shape[0]
        phase = WaypointPhase.DOWN
        curr_pos = lefmost_node_pos
        waypoints = []
        waypoints.append(curr_pos)
        while (phase != WaypointPhase.TERMINATE):
            if (phase == WaypointPhase.DOWN):
                new_pos = (curr_pos[0]+1,curr_pos[1])
                if curr_pos[0]+1 == rows and nodes[new_pos].activated:
                    waypoints.append(new_pos)
                    right_pos = return_right_position(new_pos,border_nodes)
                    if right_pos is not None:
                        waypoints.append(right_pos)
                        curr_pos = right_pos
                        phase = WaypointPhase.DOWN
                    else:
                        phase = WaypointPhase.TERMINATE
                elif nodes[new_pos].activated:
                    waypoints.append(new_pos)
                    curr_pos = new_pos
                else:
                    right_pos = return_right_position(new_pos,border_nodes)
                    if right_pos is not None:
                        waypoints.append(right_pos)
                        curr_pos = right_pos
                        phase = WaypointPhase.DOWN
                    else:
                        phase = WaypointPhase.TERMINATE
        return waypoints

           



def main():
    rows = int(input("Num rows: "))##Good value 30
    cols = int(input("Num cols: "))## Good value 30
    activation_type = input("Activation type: ")
    plt.figure()
    plt.title("Activated Nodes")
    plt.xlim(-1, rows+1)
    plt.ylim(-1, cols+1)
    plt.grid()
    nodes = np.empty([rows, cols], dtype=object)
    for i in range(rows):
        for j in range(cols):
            nodes[i,j] = Node(False,i,j)
            
    if activation_type == "rectangle":
        rec_row_start = int(input("Rectangle row start: "))## Good value 10 
        rec_row_limit = int(input("Rectangle row end: "))## Good value 20
        rec_col_start = int(input("Rectangle col start: "))## Good value 10
        rec_col_limit  =int(input("Rectangle col end: "))## Good value 20
        nodes = activate_rectangle(nodes, rec_row_start, rec_col_start, rec_row_limit, rec_col_limit)
        border_nodes,_,leftmost_node_pos, nodes = find_border_nodes(nodes)
        way_points = get_all_lawnmower_waypoints_adjustable(nodes,leftmost_node_pos,border_nodes)

    if activation_type== "circle":
        circle_center_row = int(input("Circle center row: "))## Good value 15
        circle_center_col = int(input("Circle center col: "))## Good value 15
        circle_radius = int(input("Circle radius: "))## Good value 12
        nodes = activate_circle(nodes, circle_center_row, circle_center_col, circle_radius)  
        border_nodes,_,leftmost_node_pos, nodes = find_border_nodes(nodes)
        way_points = get_all_lawnmower_waypoints_adjustable(nodes,leftmost_node_pos,border_nodes)

    if activation_type== "triangle":
        x1 = int(input("x1: "))## Good value 5
        y1 = int(input("y1: "))## Good value 5
        x2 = int(input("x2: "))## Good value 15
        y2 = int(input("y2: "))## Good value 25
        x3 = int(input("x3: "))## Good value 25
        y3 = int(input("y3: "))## Good value 5
        nodes = activate_traingle(nodes, x1,y1,x2,y2,x3,y3) 
        border_nodes,_,leftmost_node_pos, nodes = find_border_nodes(nodes)
        way_points = get_all_lawnmower_waypoints_adjustable(nodes,leftmost_node_pos,border_nodes)

    
    for i in range(rows):
        for j in range(cols):
            node = nodes[i,j]
            if node.on_border:
                plt.plot(i,j, marker = 'o',color="red")
            elif node.activated:
                plt.plot(i,j, marker = 'o',color="green")
            else:
                plt.plot(i,j, marker = 'o',color="blue")
    way_points_x = [pt[0] for pt in way_points]
    way_points_y = [pt[1] for pt in way_points]
    plt.plot(way_points_x,way_points_y,color="purple")
    plt.show()


if __name__ == "__main__":
    main()