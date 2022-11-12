import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from engine.grid import Grid

def main():

    grid=Grid(42.444250, 42.444599, -76.483682, -76.483276)

    
    rows = grid.get_num_rows()
    cols = grid.get_num_cols()
    activation_type = input("Activation type: ")
    plt.figure()
    plt.title("Activated Nodes")
    plt.xlim(-1, rows+1)
    plt.ylim(-1, cols+1)
    plt.grid()
    
    if activation_type == "rectangle":

        rec_row_start = rows // 3 ## Good value 13 
        rec_row_limit = 2 * rows // 3 ## Good value 26
        rec_col_start = cols // 3 ## Good value 11
        rec_col_limit  = 2 * cols // 3## Good value 22
        grid.activate_rectangle(rec_row_start, rec_col_start, rec_row_limit, rec_col_limit)
        grid.find_border_nodes()
        way_points = grid.get_all_lawnmower_waypoints_adjustable()

    if activation_type== "circle":
        circle_center_row = rows // 2 ## Good value 19
        circle_center_col = cols // 2 ## Good value 17
        circle_radius = min(rows, cols) // 2 ## Good value 17
        grid.activate_circle(circle_center_row, circle_center_col, circle_radius)  
        grid.find_border_nodes()
        way_points = grid.get_all_lawnmower_waypoints_adjustable()

    if activation_type== "triangle":
        x1 = rows // 6## Good value 6
        y1 = cols // 6 ## Good value 5
        x2 = rows // 2## Good value 19
        y2 = 3 * cols // 4## Good value 25 
        x3 = 3 * rows // 4## Good value 29
        y3 = cols // 6 ## Good value 5
        grid.activate_triangle(x1,y1,x2,y2,x3,y3) 
        grid.find_border_nodes()
        way_points = grid.get_all_lawnmower_waypoints_adjustable()

    if activation_type == "line":
        x = rows // 2
        y = 0
        grid.activate_line(x,y,9)
        grid.find_border_nodes()
        way_points = grid.get_all_lawnmower_waypoints_adjustable()


    for i in range(rows):
        for j in range(cols):
            node = grid.nodes[i,j]
            if node.is_border_node():
                plt.plot(i,j, marker = 'o',color="red") # Color a border
            elif node.is_active_node():
                plt.plot(i,j, marker = 'o',color="green")
            else:
                plt.plot(i,j, marker = 'o',color="blue")

    way_points_x = [pt[0] for pt in way_points]
    way_points_y = [pt[1] for pt in way_points]
    plt.plot(way_points_x,way_points_y,color="purple")
    plt.show()


if __name__ == "__main__":
    main()