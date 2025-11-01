'''
take in 2d array of data (currently 9x15)
if obstacle detected in any of boxes that aren't detecting the ground (ex: top 2/3 of boxes), then:
compute x and y position of said object (x = lateral distance from top-down perspective, y = distance ahead)
    note: (x^2 + y^2 = d^2), where d is value in a box 
    to compute x and y, account for FOV of camera, determine angle(x) and angle(y) at each box (must be precomputed),
    then use d to determine exact distances
then, given the position of the obstacle, the turning radius of the robot and the size of the robot,
determine the best possible path that minimizes the time required to get to the endpoint while
maximizing the distance from the robot to the obstacle

edge cases to consider in testing:
- very wide object; robot may try to take shortest path to avoid it repeatedly, resulting in going
  over the same area repeatedly 
- very long object; robot may avoid the head of it successfully, but then crash into its back if
  unaware of how long the object truly is 
- moving object; may require predicting the path of the object and avoiding it w.r.t. where it is in time
'''

GRID_WIDTH = 15
GRID_HEIGHT = 9
GROUND_ROWS_TO_IGNORE = 1 # potentially important if camera is low to ground, adjust later with testing
CRITICAL_DISTANCE = 2000 # 2 meters

# pretend this is the publisher node that the steering node will subscribe to
def get_obstacle_list(spatial_data: list) -> list:
    """
    Filters the 135-point spatial data list to find true, non-ground obstacles.
        note: the list is a 1d array because of the depthai function call q.get().getSpatialLocations()
    
    Returns:
        list: A list of (x, z) tuples for each detected obstacle, 
              where x is lateral distance and z is forward distance in mm.
    """
    obstacles = []
    
    if not spatial_data or len(spatial_data) != (GRID_WIDTH * GRID_HEIGHT):
        return []

    for i, loc in enumerate(spatial_data):
        coords = loc.spatialCoordinates
        
        # invalid data
        if coords.z == 0:
            continue
            
        row = i // GRID_WIDTH

        if row >= (GRID_HEIGHT - GROUND_ROWS_TO_IGNORE):
            continue
            
        if coords.z < CRITICAL_DISTANCE:
            obstacle_pos_xz = (coords.x, coords.z)
            obstacles.append(obstacle_pos_xz)
            
    return obstacles

'''
things to add when in ROS (next thing to do):

update curvature in pure pursuit algo to adjust for and avoid obstacles
according to gemini this is correct code, yet to test: 

# 1. Calculate the base curvature to follow the path
path_curv = curvature(la, self.pos, angle, LOOKAHEAD)

# 2. Calculate the avoidance curvature
avoid_curv = 0.0
K_AVOID = 1.5 # can adjust
AVOID_DISTANCE_X = 10.0 # Max distance ahead to react
AVOID_DISTANCE_Y = 2.0  # Max lateral distance to react

if self.obstacle_pos is not None:
    obs_x, obs_y = self.obstacle_pos
    
    # Only react if obstacle is within our "activation zone"
    if 0.1 < obs_x < AVOID_DISTANCE_X and abs(obs_y) < AVOID_DISTANCE_Y:
        
        # Use the same logic as pure pursuit: kappa = 2*eta / L^2
        # Here, lateral offset (eta) = obs_y
        # Lookahead (L) = obs_x
        # We add a negative sign to steer *away* from the offset
        # We use 1/x^2, so it's very strong when close
        avoid_curv = -K_AVOID * (2 * obs_y / (obs_x**2))

    # Important: Consume the obstacle data so we don't
    # react to stale information.
    self.obstacle_pos = None 

# 3. Combine the curvatures
curv = path_curv + avoid_curv
'''