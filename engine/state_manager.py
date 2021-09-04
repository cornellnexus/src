# external libraries
import enum
# internal classes
from engine.path_traversal import PathTraversal
from engine.astar import Astar
from engine.startup import Startup  # TODO
from engine.docking import Docking
from engine.docked import Docked
from engine.manual import Manual
from engine.safehold import Safehold


# if (rpi):
# else if (desktop):

class State(enum.Enum):
    Startup  # initializing software, electrical, mechanical reqs on robot
    PathTraversal  # path traversal to filter plastics
    Astar  # path traversal back to base station (rename to ReturnToBase?)
    Docking  # docking the robot at base station
    Docked  # charging the robot at base station and removing plastic
    Manual  # user controls the robot
    Safehold  # safety case where robot is not functioning as planned

    def __init__(self):
        self.current_state = None

    def set_state(self, state):
        self.current_state = state

    def get_state(self):
        return self.current_state


class StateManager:
    def execute(self):
        # TODO: probably want to initialize State somewhere outside of this function.
        state = State.get_state()

        # 1. Change state if faults exist
        # TODO: need a class that includes Faults (signal, suppress,override faults)
        # if fault, change to safehold

        # 2. Handle the states
        if (state == Startup):
            Startup.execute_startup()  # TODO: write the execute function
            # Init -> Receive user input to create grid and robot? GUI, connections established
            # TODO: Move location of initialization of simulation robot and grid (Out of path traversal, etc)
        elif (state == PathTraversal):
            PathTraversal.engine()  # TODO: change name of engine function to execute?
            # TODO: add bug algorithm to path traversal
        elif (state == Astar):
            Astar.main()  # TODO: write execute function & rename
            # TODO: add bug algorithm (for dynamic obstacles)
        elif (state == Docking):
            Docking.execute_docking()  # TODO: write execute function
        elif (state == Docked):
            Docked.execute_docked()  # TODO: write execute function
        elif (state == Manual):
            Manual.execute_manual()  # TODO: write execute function
        elif (state == Safehold):
            Safehold.execute_safehold()  # TODO: write execute function
        else:
            Safehold.execute_safehold()  # default case is safehold
