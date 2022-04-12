import matplotlib.pyplot as plt
import numpy as np

class Node:
    def __init__(self,activated,x,y):
        self.activated = activated
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

    if activation_type== "circle":
        circle_center_row = int(input("Circle center row: "))## Good value 15
        circle_center_col = int(input("Circle center col: "))## Good value 15
        circle_radius = int(input("Circle radius: "))## Good value 12
        nodes = activate_circle(nodes, circle_center_row, circle_center_col, circle_radius)  

    if activation_type== "triangle":
        x1 = int(input("x1: "))## Good value 5
        y1 = int(input("y1: "))## Good value 5
        x2 = int(input("x2: "))## Good value 15
        y2 = int(input("y2: "))## Good value 25
        x3 = int(input("x3: "))## Good value 25
        y3 = int(input("y3: "))## Good value 5
        nodes = activate_traingle(nodes, x1,y1,x2,y2,x3,y3)   


    
    for i in range(rows):
        for j in range(cols):
            node = nodes[i,j]
            if node.activated:
                plt.plot(i,j, marker = 'o',color="green")
            else:
                plt.plot(i,j, marker = 'o',color="blue")

    plt.show()


if __name__ == "__main__":
    main()