import math
from engine.phase import Phase
from robot.traversal import move_to_target_node, turn_to_target_heading

def execute_return(robot, base_loc, base_angle, allowed_docking_pos_error, allowed_heading_error, database):
        """
        Returns robot to base station when robot is in RETURN phase and switches to DOCKING.

        Arguments:
            base_loc: location of the base station in meters in the form (x, y)
            base_angle: which direction the base station is facing in terms of unit circle (in radians)
            allowed_docking_pos_error: the maximum distance in meters the robot can be from "ready to dock" position
                before it can start docking (must be small)
            allowed_heading_error: the maximum error in radians a robot can have to target heading while turning in
                place.
        """
        docking_dist_to_base = 1.0  # how close the robot should come to base before starting DOCKING
        dx = docking_dist_to_base * math.cos(base_angle)
        dy = docking_dist_to_base * math.sin(base_angle)
        target_loc = (base_loc[0] + dx, base_loc[1] + dy)

        # TODO: add obstacle avoidance support
        move_to_target_node(target_loc, allowed_docking_pos_error, database)

        # Face robot towards base station
        # TODO: probably will get rid of this
        target_heading = base_angle + math.pi
        turn_to_target_heading(target_heading, allowed_heading_error, database)

        # RETURN phase complete:
        robot.set_phase(Phase.DOCKING)



