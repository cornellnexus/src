import math
from queue import PriorityQueue

class Astar():
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
            print(len(open_set_hash))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            current = open_set.get()[2]  # Gets node
            open_set_hash.remove(current)

            # if current == end:
            #     reconstruct_path(came_from, end, draw)
            #     start.make_start()
            #     end.make_end()
            #     return True

            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1  # Since path lengths are all 1

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + \
                        heuristic(neighbor.get_pos(), end.get_pos())
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        # neighbor.make_open()
            # draw()

            # if current != start:
            #     current.make_closed()
        print("algorithm ran")

        return False
        
    def main():
        print("hi")
        ROWS = 30
        width = 600
        start_row, start_col = 2,2
        end_row, end_col = 10,10
        start = Node(start_row, start_col, width,ROWS)
        end = Node(end_row, end_col, width, ROWS)
        grid = make_grid(ROWS, width)
        algorithm(grid, start, end)


'''
A Node is a cell which composes part of the graph that our robot is localized
in.
'''
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

    def update_neighbors(self, grid):
        self.neighbors = []
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])

        # UP
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col])
        # RIGHT
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])
        # LEFT
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)

    return grid



# def get_clicked_pos(pos, rows, width):
#     gap = width // rows
#     y, x = pos
#     row = y // gap
#     col = x // gap
#     return row, col

    

# def main(win, width):
#     ROWS = 30
#     grid = make_grid(ROWS, width)

#     start = None
#     end = None

#     run = True
#     started = False

#     while run:
#         draw(win, grid, ROWS, width)
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 run = False
#             if started:
#                 continue

#             if pygame.mouse.get_pressed()[0]:
#                 pos = pygame.mouse.get_pos()
#                 row, col = get_clicked_pos(pos, ROWS, width)
#                 node = grid[row][col]
#                 if not start and node != end:
#                     start = node
#                     start.make_start()
#                 elif not end and node != start:
#                     end = node
#                     end.make_end()
#                 elif node != start and node != end:
#                     node.make_barrier()

#             if pygame.mouse.get_pressed()[2]:
#                 pos = pygame.mouse.get_pos()
#                 row, col = get_clicked_pos(pos, ROWS, width)
#                 node = grid[row][col]
#                 node.reset()
#                 if node == start:
#                     start = None
#                 if node == end:
#                     end = None

#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_SPACE and not started:
#                     for row in grid:
#                         for node in row:
#                             node.update_neighbors(grid)
#                     algorithm(lambda: draw(win, grid, ROWS, width),
#                               grid, start, end)

#                 if event.key == pygame.K_c:
#                     start = None
#                     end = None
#                     grid = make_grid(ROWS, width)

#     pygame.quit()


# main(WINDOW, WIDTH)
