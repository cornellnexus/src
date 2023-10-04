import json
from datetime import datetime

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
    
    def rmi(self):
        """
        Returns an encoded dictionary, containing the data across states.
        """

        # Create dictionary of information
        # Cast to string and encode

        # Information that we want:
        # https://docs.google.com/document/d/15H-kN6vgFRpZ-XSjbv7ulJhqrt_15Kya2FYC6WzGCtI/edit

        return json.dumps({
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "sensors": {
                "gps": {
                    "connected": self.robot_state.gps_connected,
                    "reading": json.dumps(self.robot_state.gps.get_gps())
                },
                "imu": {
                    "connected": self.robot_state.imu_connected,
                    "reading": json.dumps(self.robot_state.imu.get_imu())
                },
                # "break_beams": {
                #     "half1": {
                #         "connected": 
                #         "blocked": 
                #     },
                #     "half2": {
                #         "connected": 
                #         "blocked": 
                #     },
                #     "full1": {
                #         "connected": 
                #         "blocked": 
                #     },
                #     "full2": {
                #         "connected": 
                #         "blocked": 
                #     },
                #     #"half_full": 
                #     #"max_full": 
                # },
                "wheel_motors": {
                    #"duty_cycle": Float,
                    #"linear_velocity": Float,
                    #"left_wheel_velocity": Float,
                    #"right_wheel_velocity": Float
                },
                "cams": {
                    "front_cam": {
                        #"connected": Boolean,
                        #"tag_id_detected": String or null
                    },
                    "back_cam": {
                        #"connected": Boolean,
                        #"tag_id_detected": String or null
                    }
                },
                "ultrasonic": { 
                    "front_uls": {
                        #"connected": Boolean,
                        #"distance_to_object": Float or null
                    },
                    "left1_uls": {
                        #"connected": Boolean,
                        #"distance_to_object": Float or null
                    },
                    "left2_uls": {
                       # "connected": Boolean,
                       # "distance_to_object": Float or null
                    },
                    "right_uls": {
                       # "connected": Boolean,
                       # "distance_to_object": Float or null
                    }
                }, 
            }
            "battery": {
                "battery_percent": self.robot_state.battery,
                # "time_until_recharge": 
                # "low_power_mode":
                
            },
            
            "metrics": {
                "goal_loc": self.robot_state.goal_location,
                "state": self.robot_state.goal_location,
                "phase": self.robot_state.phase,
                "eta_to_base": String HH:MM:SS,
                "goal_nodes_completed": Integer,
                "eta_complete": 
            },
        }
        )
    
    