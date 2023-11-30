import math
from csv_files.csv_util import write_phase_to_csv


def phase_change(robot_state):
    if robot_state.should_store_data:
        write_phase_to_csv(robot_state.phase)


def calculate_dist(init_pos, final_pos):
    return math.sqrt(
        (final_pos[0] - init_pos[0]) ** 2 + (final_pos[1] - init_pos[1]) ** 2
    )
