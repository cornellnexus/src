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
        self.rmi()
    
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
                    "reading": json.dumps(self.robot_state.gps.get_gps()),
                },
                "imu": {
                    "connected": self.robot_state.imu_connected,
                    "reading": json.dumps(self.robot_state.imu.get_imu()),
                },
                # need to wait for commits
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
                    "duty_cycle": self.robot_state.motor_controller.dc,
                    "linear_velocity": self.robot_state.linear_v, 
                    "left_wheel_velocity": self.robot_state.motorcontroller.left_right_angular_vel(self.robot_state.angular_v, self.robot_state.linear_v)[0],
                    "right_wheel_velocity": self.robot_state.motorcontroller.left_right_angular_vel(self.robot_state.angular_v, self.robot_state.linear_v)[1]
                },
                # need to wait for commits
                #"cams": {
                    #"front_cam": {
                        #"connected": Boolean,
                        #"tag_id_detected": String or null
                    #},
                    #"back_cam": {
                        #"connected": Boolean,
                        #"tag_id_detected": String or null
                    #}
                #},
                "ultrasonic": {
                    "front_uls": 
                    {
                        "connected": (self.robot_state.front_ultrasonic.distance() >= max (self.robot_state.front_sensor_offset, 0)),
                        "distance_to_object": self.robot_state.front_ultrasonic.distance(),
                    },
                    "left1_uls": 
                    {
                        "connected": self.robot_state.lf_ultrasonic.distance() >= 0,
                        "distance_to_object": self.robot_state.lf_ultrasonic.distance(),
                    },
                    "left2_uls": 
                    {
                       "connected": self.robot_state.lb_ultrasonic.distance() >= 0,
                       "distance_to_object": self.robot_state.lb_ultrasonic.distance(),
                    },
                    "right1_uls": 
                    {
                       "connected": self.robot_state.rf_ultrasonic.distance() >= 0,
                       "distance_to_object": self.robot_state.rf_ultrasonic.distance(),
                    },
                    "right2_uls": 
                    {
                       "connected": self.robot_state.rb_ultrasonic.distance() >= 0,
                       "distance_to_object": self.robot_state.rb_ultrasonic.distance(),
                    }
                }
            },
            "battery": {
                "battery_percent": self.robot_state.battery,
                "time_until_recharge": "00:00:00", #TODO 
                "low_power_mode": False, #TODO
            },
            "metrics": {
                "goal_loc": {
                    "global_coord": self.robot_state.goal_location,  #TODO
                    "local_coord": self.robot_state.goal_location,
                    "robot_goal_dist": 0, #TODO Float <Distance between robot and goal node>
                },
                "state":{
                    "global_coord": self.robot_state.state[0], #TODO
                    "local_coord": self.robot_state.state[0],
                    "heading": self.robot_state.state[1]
                },
                "phase": self.robot_state.phase,
                "eta_to_base": "00:00:00", #TODO
                "goal_nodes_completed": 0, #TODO 
                "eta_complete": "00:00:00", #TODO
                "eta_next_node": "00:00:00", #TODO
            },
        })
    
    