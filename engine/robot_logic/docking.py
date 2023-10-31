from engine.phase import Phase
from engine.robot_logic.robot_helpers import phase_change


def docking_logic(robot_state):
    # TODO: add traverse to doc at base station in case mission didnt finish
    robot_state.goal_location = None
    robot_state.prev_phase = Phase.DOCKING
    robot_state.phase = Phase.COMPLETE
    phase_change(robot_state)  # temporary for simulation purposes
