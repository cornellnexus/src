import numpy as np
import math 
from vincenty import vincenty
from haversine import haversine, Unit


MAX_V = 5
WHEEL_TO_CENTER = 0.2
EARTH_RADIUS = 6371008.8 # meters

def robot_to_global(pose,x_robot,y_robot):
    """
    Transforms the (x_robot, y_robot) point in robot coordinates into global 
    coordinates (x_global, y_global). Outputs a [2x1] np array.

    Inputs: 
        pose: robot's current pose (global) [3x1]
        x_R: x coordinate in robot/body frame 
        y_R: y coordinate in robot/body frame
    """
    px = pose[0]
    py = pose[1]
    theta = pose[2]
    Tgr = np.array([[math.cos(theta), -1*math.sin(theta), px], \
                    [math.sin(theta), math.cos(theta), py], \
                    [0, 0, 1] ])
    pose_global = Tgr @ np.array([[x_robot],[y_robot]])
    return pose_global[0:2]

def global_to_robot(pose, xy_global):
    """
    Transforms xy_global into robot coordinates. Outputs a [2x1] np array. 
    
    Inputs: 
        pose: robot's current pose (global) [3x1]
        xy_global: 2D point in gloabl coordinates [2x1]
    """ 
    px = pose[0]
    py = pose[1]
    theta = pose[2]
    Tgr = np.array([[math.cos(theta), -1*math.sin(theta), px], \
                      [math.sin(theta), math.cos(theta), py], \
                      [0, 0, 1] ])
    Trg = np.linalg.inv(Tgr)
    pose_robot = Trg * xy_global
    return pose_robot[0:2]

def feedback_lin(curr_pose, vx_global, vy_global, epsilon):
    """
    Given desired x and y velocity in the inertial frame (calculated by 
    desired pose - current pose), returns the linear and angular 
    velocity the robot should move at.
        curr_pose: [3x1] (is this global?)
    """
    theta = float(curr_pose[2])
    v = vx_global * math.cos(theta) + vy_global * math.sin(theta)
    w = (-1/epsilon) * vx_global * math.sin(theta) + (1/epsilon) * vy_global * math.cos(theta)
    return (v,w)
# test_state = np.array([[0],[0],[math.pi/2]])
# print(feedback_lin(test_state, 5, 10, 0.1))


def limit_cmds(v, w, max_v, wheel_to_center):
    '''
    Returns a scaled v and w, such that 
    '''
    diameter = wheel_to_center * 2
    v_leftwheel = (-w * diameter + 2*v)/2
    v_rightwheel = (w*diameter + 2*v)/2

    if abs(v_leftwheel) > max_v and abs(v_rightwheel) <= max_v:
        alpha = max_v/abs(v_leftwheel)
    elif abs(v_leftwheel) <= max_v and abs(v_rightwheel) > max_v:
        alpha = max_v/abs(v_rightwheel)   
    elif abs(v_leftwheel) > max_v and abs(v_rightwheel) > max_v:
        alpha = min(max_v/abs(v_rightwheel),max_v/abs(v_leftwheel))
    else:
        alpha = 1
        
    v_leftwheel = v_leftwheel * alpha
    v_rightwheel = v_rightwheel * alpha
    scaled_v = (1/2)*(v_leftwheel + v_rightwheel)
    scaled_w = (1/diameter)*(v_rightwheel-v_leftwheel)

    return (scaled_v, scaled_w)

def integrate_odom(pose, d, phi):
    if phi == 0:
        new_x = pose[0] + d * math.cos(pose[2])
        new_y = pose[1] + d * math.sin(pose[2])
        new_theta = pose[2]
    else:
        new_x = pose[0] + (d/phi) * (math.sin(pose[2]+phi) - math.sin(pose[2]))
        new_y = pose[1] + (d/phi) * (-math.cos(pose[2]+phi) + math.cos(pose[2]))
        new_theta = (pose[2] + phi) % (2*math.pi)
        # new_theta = (pose[2] + phi)
    return np.array([[float(new_x)], [float(new_y)], [float(new_theta)]])

def meters_to_gps(lat,long, dy, dx):
    """
    Returns the approximate gps coordinate of starting at (lat,long) and moving 
    dx meters horizontally (E-W) and dy meters vertically. [N-S]
    """
    new_lat = lat  + ((dy / EARTH_RADIUS) * (180 / math.pi))/1000
    new_long = long + ((dx / EARTH_RADIUS) * (180 / math.pi) / math.cos(lat * math.pi/180))
    return new_lat,new_long

def meters_to_lat(m):
    """
    Returns an approximation of m meters in units of latitude.
    """
    lat = ((m / EARTH_RADIUS) * (180 / math.pi))
    return lat

def meters_to_long(m,latitude):
    """
    Returns an approximation of m meters in units of longitude.
    """
    long = ((m / EARTH_RADIUS) * (180 / math.pi) / math.cos(latitude * math.pi/180))
    return long

def get_vincenty_x(coord1, coord2):
    """
    Calculates the longitudinal vincenty distance btw gps coordinates [coord1] 
    and [coord2] in meters. 
    """
    coord1_x = (coord2[0],coord1[1])
    x = vincenty(coord1_x,coord2)*1000
    return x

def get_vincenty_y(coord1, coord2):
    """
    Calculates the latitude vincenty distance btw gps coordinates [coord1] and 
    [coord2] in meters. 
    """
    coord1_y = (coord1[0],coord2[1])
    y = vincenty(coord1_y,coord2)*1000
    return y

def get_haversine_x(coord1, coord2):
    """
    Calculates the longitudinal haversine distance btw gps coordinates [coord1] 
    and [coord2] in meters. 
    """
    coord1_x = (coord2[0],coord1[1])
    x = haversine(coord1_x,coord2,unit = Unit.METERS)
    return x

def get_haversine_y(coord1, coord2):
    """
    Calculates the latitude vincenty distance btw gps coordinates [coord1] 
    and [coord2] in meters. 
    """
    coord1_y = (coord1[0],coord2[1])
    y = haversine(coord1_y,coord2,unit = Unit.METERS)
    return y



