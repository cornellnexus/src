#internal classes
from engine.path_traversal import PathTraversal
from engine.astar import Astar
from engine.startup import Startup 
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
    self.state_obj = State()
    self.state = self.state_obj.get_state()
    
    

    #1. Change state if faults exist 
    # TODO: need a class that includes Faults (signal, suppress,override faults)
    # if fault, change to safehold 

    #2. Handle the states
    if (self.state == "startup"):
      Startup.execute_startup() 
      # Init -> Receive user input to create grid and robot? GUI, connections established 
    elif (self.state == "path_traversal"):
      PathTraversal.engine() #TODO: change name of engine function to execute?
      #TODO: add bug algorithm to path traversal 
    elif (self.state == "astar"):
      Astar.main() #TODO: write execute function & rename
      #TODO: add bug algorithm (for dynamic obstacles)
    elif (self.state == "docking"):
      Docking.execute_docking() #TODO: write execute function 
    elif (self.state == "docked"):
      Docked.execute_docked() #TODO: write execute function 
    elif (self.state == "manual"):
      Manual.execute_manual() #TODO: write execute function 
    elif (self.state == "safehold"): 
      Safehold.execute_safehold() #TODO: write execute function 
    else: 
      Safehold.execute_safehold() #default case is safehold