class Transmittor:
    """
    A class that will transmit the current robot and mission states to the GUI
    """
    
    def __init__(self, robot_state, mission_state):
        """
        Store references to states. 

        Parameters:
            robot_state: an instance encapsulating conditions, measurements, etc. (i.e. all data) for a Robot instance
            mission_state: an instance encapsulating conditions, measurements, etc. (i.e. all data) for a Mission instance
        """
        self.robot_state = robot_state
        self.mission_state = mission_state
                 
    def transmit_to_gui(self):
        """
        Call this function to transmit information to the GUI
        """        
        self.gather_info()
    
    def gather_info(self):
        """
        Returns an encoded dictionary, containing the data across states.
        """

        # Create dictionary of information
        # Cast to string and encode

        # Information that we want:
        # https://docs.google.com/document/d/15H-kN6vgFRpZ-XSjbv7ulJhqrt_15Kya2FYC6WzGCtI/edit

        pass
    
    