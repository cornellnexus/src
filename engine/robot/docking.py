from engine.phase import Phase
def execute_docking(robot):
    # TODO: add traverse to doc at base station in case mission didnt finish
    robot.set_phase(Phase.COMPLETE)  # temporary for simulation purposes