#internal classes
from engine.path_traversal import PathTraversal
from engine.astar import Astar
from engine.startup import Startup #TODO
from engine.docking import Docking 
from engine.docked import Docked 
from engine.manual import Manual 
from engine.safehold import Safehold

# if (rpi): 
# else if (desktop):

class State():
  def __init__(self): 
      self.states = {
        "startup": Startup, 
        "path_traversal": PathTraversal, 
        "astar": Astar, 
        "docking": Docking, 
        "docked": Docked, 
        "manual": Manual, 
        "safehold": Safehold 
      }

      self.current_state = None

  def set_state(self, state): 
    self.current_state = state

  def get_state(self): 
    return self.current_state

class StateManager: 
  def execute(self): 
    state_obj = State()
    state = state_obj.get_state()

    #1. Change state if faults exist 
    # TODO: need a class that includes Faults (signal, suppress,override faults)
    # if fault, change to safehold 

    #2. Handle the states
    if (state == "startup"):
      Startup.execute_startup() #TODO: write the execute function
      # Init -> Receive user input to create grid and robot? GUI, connections established 
      #TODO: Move location of initialization of simulation robot and grid (Out of path traversal, etc)
    elif (state == "path_traversal"):
      PathTraversal.engine() #TODO: change name of engine function to execute?
      #TODO: add bug algorithm to path traversal 
    elif (state == "astar"):
      Astar.main() #TODO: write execute function & rename
      #TODO: add bug algorithm (for dynamic obstacles)
    elif (state == "docking"):
      Docking.execute_docking() #TODO: write execute function 
    elif (state == "docked"):
      Docked.execute_docked() #TODO: write execute function 
    elif (state == "manual"):
      Manual.execute_manual() #TODO: write execute function 
    elif (state == "safehold"): 
      Safehold.execute_safehold() #TODO: write execute function 
    else: 
      Safehold.execute_safehold() #default case is safehold