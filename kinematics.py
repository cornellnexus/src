import numpy as np
import math 

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
    """
    theta = float(curr_pose[2])
    v = vx_global * math.cos(theta) + vy_global * math.sin(theta)
    w = (-1/epsilon) * vx_global * math.sin(theta) + (1/epsilon) * v * math.cos(theta)
    return np.array([v],[w])


def limit_cmds(v, w, max_v, wheel_to_center):
    '''
    Returns a scaled v and w, such that 
    '''
    diameter = wheel_to_center * 2
    v_leftwheel = (-w * diameter + 2*v)/2
    v_rightwheel = (w*diameter + 2*v)/2

    if abs(v_leftwheel) >= max_v and abs(v_rightwheel) <= max_v:
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

#robot_to_wheel
print('hi')