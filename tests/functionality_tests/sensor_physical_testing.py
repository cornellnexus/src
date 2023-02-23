from engine.robot_state import Robot_State
from engine.robot import Robot 
from electrical.gps import GPS 
from engine.grid import Grid 
from engine. database import DataBase
import math

if __name__ == "__main__":
    
    test_state = 0 # 0 is no node straight, 1 is node straight, 2 is turn, -1 is sensor
    rb_state = Robot_State(0, 0, 0, .2, .5, .2)
    rb = Robot(rb_state)
    database = DataBase(rb)

    if test_state == -1:
      print("gps")
      print(rb_state.gps.get_gps())
      magnetometer = rb_state.imu.get_imu().magnetic
      heading = math.atan2(magnetometer[1], magnetometer[0]) * 180 / math.pi
      print("heading")
      print(heading)
      print()

    elif test_state == 0:
      rb.move_to_target_node([10,10], 2, database)

    elif test_state == 1:
      grid = Grid (42.444250, 42.444599, -76.483682, -76.483276) # min lat and long is origin (0,0)
      waypoints = grid.get_straight_line_waypoints(y_start_pct=2)
      while len (waypoints) > 0:
        curr_waypoint = waypoints[0].get_m_coords()
        rb.move_to_target_node(curr_waypoint, 2, database)
        waypoints.popleft()

    elif test_state == 2:
      rb.turn_to_target_heading(math.pi, 2, database)

# next test mission
# next tune PID