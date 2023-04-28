# THIS IS A TESTING SCRIPT. USE THIS SCRIPT WHEN PHYSICALLY TESTING THE ROBOT AND CHANGE THE test_state TO THE STATE YOU NEED

from engine.robot_state import Robot_State
from engine.robot import Robot
from electrical.gps import GPS
from engine.grid import Grid
from engine. database import DataBase
import math

if __name__ == "__main__":
    # make sure to go to imu and gps and set import to true
    # 0 is no node straight, 1 is node straight (generating node with grid), 2 is turn, -1 is sensor
    test_state = 0
    rb_state = Robot_State(0, 0, 0, .2, .5, .2)
    r2d2 = Robot(rb_state)
    database = DataBase(r2d2)
    r2d2.execute_setup()
    rb_state.self.rb_state.enable_obstacle_avoidance = False

    if test_state == -1:
        print("gps", rb_state.gps.get_gps())
        magnetometer = rb_state.imu.get_imu().get("mag")
        heading = math.atan2(magnetometer[1], magnetometer[0]) * 180 / math.pi
        print("heading", heading)
        print()

    elif test_state == 0:
        r2d2.move_to_target_node([10, 10], 2, database)

    elif test_state == 1:
        # min lat and long is origin (0,0)
        grid = Grid(42.444250, 42.444599, -76.483682, -76.483276)
        waypoints = grid.get_straight_line_waypoints(y_start_pct=2)
        while len(waypoints) > 0:
            curr_waypoint = waypoints[0].get_m_coords()
            r2d2.move_to_target_node(curr_waypoint, 2, database)
            waypoints.popleft()

    elif test_state == 2:
        r2d2.turn_to_target_heading(math.pi, 2, database)

# next test mission
# next tune PID
