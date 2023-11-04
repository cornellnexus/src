# THIS IS A TESTING SCRIPT FOR TURN_TO_TARGET_HEADING. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE test_state TO THE STATE YOU NEED

from engine.robot_state import Robot_State
from engine.robot import Robot
from engine.robot_logic.traversal import simpler_turn_to_target_heading
from engine.database import DataBase
import math

if __name__ == "__main__":
    # 0 does nothing, 1 is a 90 degree turn, 2 is a 180 degree turn, 3 is a -90 degree turn, 4 is a negative 180 degree turn
    test_state = 0
    rb_state = Robot_State(0, 0, 0, 0.2, 0.5, 0.2)
    r2d2 = Robot(rb_state)
    database = DataBase(r2d2)
    r2d2.execute_setup()
    rb_state.enable_obstacle_avoidance = False

    if test_state == 0:
        simpler_turn_to_target_heading(rb_state, 0, 2, database)

    elif test_state == 1:
        simpler_turn_to_target_heading(rb_state, math.pi / 2, 2, database)

    elif test_state == 2:
        simpler_turn_to_target_heading(rb_state, math.pi, 2, database)
    
    elif test_state == 3:
        simpler_turn_to_target_heading(rb_state, -math.pi / 2, 2, database)
    
    elif test_state == 4:
        simpler_turn_to_target_heading(rb_state, -math.pi, 2, database)
