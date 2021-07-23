#external libraries
import enum 
#internal classes
from engine.path_traversal import PathTraversal
from engine.astar import Astar

#TODO: should this file be outside of Engine folder?? 

# if (rpi): 
# else if (desktop):

class State(enum.Enum):
  Startup #initializing software, electrical, mechanical reqs on robot
  PathTraversal #path traversal to filter plastics
  Astar #path traversal back to base station (rename to ReturnToBase?)
  Docking #docking the robot at base station
  Docked #charging the robot at base station and removing plastic
  Manual #user controls the robot
  Safehold #safety case where robot is not functioning as planned 

  def __init__(self): 
    self.current_state = PathTraversal #TODO: change this to None
  
  def get_state(self): 
    return self.current_state

class StateManagement: 
  def execute(self): 
    #TODO: probably want to initialize State somewhere outside of this function.
    state = State.get_state()

    #1. Change state if faults exist 
    # TODO: need a class that includes Faults (signal, suppress,override faults)
    # if fault, change to safehold 

    #2. Handle the states
    if (state == State.Startup):
      State.Startup.execute_startup() #TODO: write the execute function
      # Init -> Receive user input to create grid and robot? GUI, connections established 
      #TODO: Move location of initialization of simulation robot and grid (Out of path traversal, etc)
    elif (state == State.PathTraversal):
      State.PathTraversal.engine() #TODO: change name of engine function to execute?
      #TODO: add bug algorithm to path traversal 
    elif (state == State.Astar):
      State.Astar.main() #TODO: write execute function & rename
      #TODO: add bug algorithm (for dynamic obstacles)
    elif (state == State.Docking):
      State.Docking.execute_docking() #TODO: write execute function 
    elif (state == State.Docked):
      State.Docked.execute_docked() #TODO: write execute function 
    elif (state == State.Manual):
      State.Manual.execute_manual() #TODO: write execute function 
    elif (state == State.Safehold): 
      State.Safehold.execute_safehold() #TODO: write execute function 
    else: 
      State.Safehold.execute_safehold() #default case is safehold