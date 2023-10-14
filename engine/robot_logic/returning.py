import math
from engine.phase import Phase
from engine.robot_logic.robot_helpers import phase_change
from engine.robot_logic.traversal import move_to_target_node, turn_to_target_heading


def return_logic(robot_state, mission_state, database):
    """
    Returns robot to base station when robot is in RETURN phase and switches to DOCKING.

    Arguments:
        base_loc: location of the base station in meters in the form (x, y)
        base_angle: which direction the base station is facing in terms of unit circle (in radians)
        allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
            before it can start docking (must be small)
        allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
            place.
    """
    robot_state.prev_phase = Phase.RETURN
    docking_dist_to_base = (
        1.0  # how close the robot should come to base before starting DOCKING
    )
    dx = docking_dist_to_base * math.cos(mission_state.base_station.heading)
    dy = docking_dist_to_base * math.sin(mission_state.base_station.heading)
    target_loc = (
        mission_state.base_station.position[0] + dx,
        mission_state.base_station.position[1] + dy,
    )

    robot_state.goal_location = target_loc
    move_to_target_node(
        robot_state, target_loc, mission_state.allowed_docking_pos_error, database
    )

    # Face robot towards base station
    # TODO: probably will get rid of this
    target_heading = mission_state.base_station.heading + math.pi
    turn_to_target_heading(
        robot_state, target_heading, mission_state.allowed_heading_error, database
    )

    # RETURN phase complete:
    robot_state.phase = Phase.DOCKING
    phase_change(robot_state)
