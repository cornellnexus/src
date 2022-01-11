import os
#run using <python -m gui.together>


os.system("python -m gui.gui &")
os.system("python -m gui.retrieve_inputs &")
os.system("python -m engine.sim_trajectory")






