from queue import PriorityQueue


# TODO: documentation
class Astar:
    # A* Algorithm (hardcoded inputs)
    # Hardcoded: ROWS, SCREEN WIDTH, STARTING NODE, ENDING NODE
    def make_grid(rows, width):
        grid = []
        gap = width // rows
        for i in range(rows):
            grid.append([])
            for j in range(rows):
                node = Node(i, j, gap, rows)
                grid[i].append(node)
        return grid

    def heuristic(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x1 - x2) + abs(y1 - y2)

    def algorithm(grid, start, end):
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start))
        came_from = {}
        g_score = {node: float("inf") for row in grid for node in row}
        g_score[start] = 0
        f_score = {node: float("inf") for row in grid for node in row}
        f_score[start] = heuristic(start.get_pos(), end.get_pos())

        # Mirrors open_set, is used to check if something is inside the queue
        open_set_hash = {start}

        while not open_set.empty():
            current = open_set.get()[2]  # Gets node
            open_set_hash.remove(current)

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1  # Since path lengths are all 1

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + heuristic(
                        neighbor.get_pos(), end.get_pos()
                    )
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
        return False

    def main():
        ROWS = 30
        width = 600
        start_row, start_col = 2, 2
        end_row, end_col = 10, 10
        start = Node(start_row, start_col, width, ROWS)
        end = Node(end_row, end_col, width, ROWS)

        grid = make_grid(ROWS, width)

        algorithm(grid, start, end)


"""
A Node is a cell which composes part of the graph that our robot is localized
in.
"""


class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col
