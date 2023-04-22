from engine.phase import Phase
from engine.robot_logic.robot_helpers import set_phase
def execute_docking(robot):
    # TODO: add traverse to doc at base station in case mission didnt finish
    robot = set_phase(robot, Phase.COMPLETE)  # temporary for simulation purposes