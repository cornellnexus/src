from engine.robot_state import Robot_State
from engine. robot import Robot 
from electrical.gps import GPS 
from engine.grid import Grid 
from engine. database import DataBase

if __name__ == "__main__":
    rb_state = Robot_State(0, 0, 0, .2, .5, .2)
    rb = Robot (rb_state)
    database = DataBase(rb)
    rb.move_to_target_node([10,10], 2, database)
    # grid = Grid (42.444250, 42.444599, -76.483682, -76.483276) # min lat and long is origin (0,0)
    # waypoints = grid.get_straight_line_waypoints(y_start_pct=2)
    # while len (waypoints) > 0:
        # curr_waypoint = waypoints[0].get_m_coords()
        # rb.move_to_target_node(curr_waypoint, 2, database)

# if not work, test GPS and IMU only 
# when work test using nodes
# next test turn
# next test mission
# next tune PID