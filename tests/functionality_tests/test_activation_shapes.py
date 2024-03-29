import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from engine.grid import Grid


def main():
    grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)

    rows = grid.get_num_rows()
    cols = grid.get_num_cols()

    if rows <= 2 and cols >= 2:
        h = rows
        rows = cols
        cols = h
        print("flip rows, cols for traversal")

    activation_type = input("Activation type: ")
    direction = input("Traversal Direction: ")
    is_vertical = True
    direction = direction.upper()

    if direction == "UP":
        grid.direction = grid.Direction.UP
    elif direction == "DOWN":
        grid.direction = grid.Direction.DOWN
    elif direction == "RIGHT":
        grid.direction = grid.Direction.RIGHT
        is_vertical = False
    elif direction == "LEFT":
        grid.direction = grid.Direction.LEFT
        is_vertical = False
    else:
        raise ValueError("Invalid direction")
    plt.figure()
    plt.title("Activated Nodes")
    plt.xlim(-1, grid.get_num_rows() + 1)
    plt.ylim(-1, grid.get_num_cols() + 1)
    plt.grid()

    if activation_type == "grid":
        rec_row_start = 0
        rec_row_limit = rows
        rec_col_start = 0
        rec_col_limit = cols
        grid.activate_rectangle(
            rec_row_start, rec_col_start, rec_row_limit, rec_col_limit
        )
        grid.find_border_nodes()
        # way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(direction)
        way_points = grid.simple_get_guided_waypoints()

    if activation_type == "rectangle":
        rec_row_start = rows // 3  # Good value 13
        rec_row_limit = 2 * rows // 3  # Good value 26
        rec_col_start = cols // 3  # Good value 11
        rec_col_limit = 2 * cols // 3  # Good value 22
        grid.activate_rectangle(
            rec_row_start, rec_col_start, rec_row_limit, rec_col_limit
        )
        grid.find_border_nodes()
        # way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(direction)
        way_points = grid.simple_get_guided_waypoints()

    if activation_type == "circle":
        circle_center_row = rows // 2  # Good value 19
        circle_center_col = cols // 2  # Good value 17
        circle_radius = min(rows, cols) // 2  # Good value 17
        grid.activate_circle(circle_center_row, circle_center_col, circle_radius)
        grid.find_border_nodes()
        # way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(direction)
        way_points = grid.simple_get_guided_waypoints()
    # TODO: Vertical traversal in triangle
    # The triangle test is currently specific for horizontal traversal, and does not work for vertical traversal due to its orientation
    if activation_type == "triangle":
        x1 = rows // 6  # Good value 6
        y1 = cols // 6  # Good value 5
        x2 = rows // 2  # Good value 19
        y2 = 3 * cols // 4  # Good value 25
        x3 = 3 * rows // 4  # Good value 29
        y3 = cols // 6  # Good value 5
        grid.activate_triangle((x1, y1), (x2, y2), (x3, y3))
        grid.find_border_nodes()
        # way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(direction)
        way_points = grid.simple_get_guided_waypoints()
    if activation_type == "line":
        row = 0  # rows are y position
        col = cols // 2  # cols are x position
        grid.activate_line((row, col), n=cols // 2, is_vertical=is_vertical)
        grid.find_border_nodes()
        way_points = grid.simple_get_guided_waypoints()
        # way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(
        #     direction)

    if activation_type == "parallelogram":
        rec_row_start = rows // 4
        rec_row_limit = 3 * rows // 4
        rec_col_start = 4
        grid.activate_parallelogram(
            rec_row_start, rec_col_start, rec_row_limit
        )
        grid.find_border_nodes()
        way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(is_vertical)

    if activation_type == "hexagon":
        row = rows // 2
        col = cols // 2
        radius = 6
        grid.activate_hexagon((row, col), radius)
        grid.find_border_nodes()
        way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(is_vertical)
    
    if activation_type == "snake":
        row = rows // 3
        col = cols // 3
        row_limit = 2 * rows // 3
        col_limit = 2 * cols // 3
        spacing = 3
        grid.activate_snake((row, col), (row_limit, col_limit), spacing)
        grid.find_border_nodes()
        way_points = grid.get_all_guided_lawnmower_waypoints_adjustable(is_vertical)

    # Questions: what if we make a new file for visualizing the path? It would also help with catching exceptions.
    #  For example, `node = grid.nodes[i,j]` can throw index out of bound error given STEP_SIZE_METERS of 100,
    #   num rows and cols 3 and circle activation type because num_y_step and num_x_step in int and inversely
    #   proportional to STEP_SIZE_METERS, so num_y_step and num_x_step is 0. Same problem when inputting row and col.
    #   Not sure the formula for determining the max row and col to input tho
    for i in range(grid.get_num_rows()):
        for j in range(grid.get_num_cols()):
            node = grid.nodes[i, j]
            if node.is_border_node():
                plt.plot(i, j, marker="o", color="red")  # Color a border
            elif node.is_active_node():
                plt.plot(i, j, marker="o", color="green")
            else:
                plt.plot(i, j, marker="o", color="blue")

    way_points_x = [pt.y for pt in way_points]
    way_points_y = [pt.x for pt in way_points]
    plt.plot(way_points_x, way_points_y, color="purple")
    plt.show()


if __name__ == "__main__":
    main()