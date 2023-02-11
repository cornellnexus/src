import math
import time
from PIL import Image

import pygame
from gui.boundary_following_gui import Graphics, Robot, Ultrasonic
from constants.definitions import GUI_DIR

def draw_constants(gfx, rbt, start, end):
    # draw blank canvas
    gfx.map.blit(gfx.map_img, (0, 0))

    # Adding start surface (blue box)
    startSurf = pygame.Surface(rbt.size)
    startSurf.fill(gfx.blue)
    # subtract by half robot size bc robot start is the center of the robot
    gfx.map.blit(startSurf, (start[0] - rbt.size[0] / 2, start[1] - rbt.size[1] / 2))

    # Adding end surface (red box)
    endSurf = pygame.Surface(rbt.size)
    endSurf.fill(gfx.red)
    gfx.map.blit(endSurf, (end[0], end[1]))

    # Adding green following line to the end + half the robot size to get to the middle ending square
    pygame.draw.line(gfx.map, gfx.green, start, (end[0] + rbt.size[0] / 2, end[1] + rbt.size[1] / 2))

def update_viz(gfx, rbt, start, end, robot):
    draw_constants(gfx, rbt, start, end)
    gfx.draw_robot(robot.x, robot.y, robot.heading)

def draw_sensor_data(debug, gfx, point_cloud, point_clouds, ultrasonics):
    # Draw point cloud and update screen
    if debug:
        gfx.draw_sensor_data(point_cloud)
        gfx.draw_side_sensor_data(point_clouds)
    else:
        for ultrasonic in ultrasonics:
            point = ultrasonic.pt
            if point is not None:
                gfx.draw_pt(point)
                line_clr = gfx.green
                if not ultrasonic.is_side:
                    line_clr = gfx.red
                pygame.draw.line(gfx.map, line_clr, ultrasonic.pos, point[0])

def update_ultrasonics(robot, ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom):
    # Get ultrasonic current position
    sensor_directions = robot.side_sensor_angle()
    ultrasonic_lt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
    ultrasonic_lb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])
    ultrasonic_rt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
    ultrasonic_rb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])
    
    # Update ultrasonic positions
    ultra_sonic.update_pos((robot.x, robot.y))
    ultra_sonic_left_top.update_pos(ultrasonic_lt_loc)
    ultra_sonic_left_bottom.update_pos(ultrasonic_lb_loc)
    ultra_sonic_right_top.update_pos(ultrasonic_rt_loc)
    ultra_sonic_right_bottom.update_pos(ultrasonic_rb_loc)

    # Read sensor data and create a point cloud
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    point_cloud_LT = ultra_sonic_left_top.side_sense_obstacles(ultrasonic_lt_loc, robot.heading + math.pi / 2)
    point_cloud_LB = ultra_sonic_left_bottom.side_sense_obstacles(ultrasonic_lb_loc, robot.heading + math.pi / 2)
    point_cloud_RT = ultra_sonic_right_top.side_sense_obstacles(ultrasonic_lb_loc, robot.heading - math.pi / 2)
    point_cloud_RB = ultra_sonic_right_bottom.side_sense_obstacles(ultrasonic_rb_loc, robot.heading - math.pi / 2)

    # Finds pt with minimum distance to obstacle for each ultrasonnics sensor
    pt = ultra_sonic.min_distance((robot.x, robot.y), point_cloud)
    pt_LT = ultra_sonic_left_top.min_distance(ultrasonic_lt_loc, point_cloud_LT)
    pt_LB = ultra_sonic_left_bottom.min_distance(ultrasonic_lb_loc, point_cloud_LB)
    pt_RT = ultra_sonic_right_top.min_distance(ultrasonic_rt_loc, point_cloud_RT)
    pt_RB = ultra_sonic_right_bottom.min_distance(ultrasonic_rb_loc, point_cloud_RB)

    # Update pt attribute in each ultrasonic obj
    ultra_sonic.update_pt(pt)
    ultra_sonic_left_top.update_pt(pt_LT)
    ultra_sonic_left_bottom.update_pt(pt_LB)
    ultra_sonic_right_top.update_pt(pt_RT)
    ultra_sonic_right_bottom.update_pt(pt_RB)

    draw_sensor_data(debug, gfx, point_cloud, [point_cloud_LT, point_cloud_LB, point_cloud_RT, point_cloud_RB], 
    [ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top,
            ultra_sonic_right_bottom])

    obstacleDetected = False
    if robot.detect_obstacles(point_cloud):
        obstacleDetected = True

    return ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom, obstacleDetected

debug = False
MAP_DIMENSIONS = (600, 1200)

start_positions = [(100, 160), (100, 300), (100, 100), (400, 450),
                   (600, 450), (100, 160), (100, 160), (800, 150)]
end_positions = [(800, 150), (700, 150), (900, 150), (800, 150),
                 (800, 150), (400, 450), (100, 160), (100, 160)]
maps = ["boundary_map", "boundary_map", "boundary_map", "boundary_map", 
        "boundary_map", "boundary_map", "boundary_map", "boundary_map"]
# maps = ["blank", "boundary_map", "roundy_maps", "obstacaly_map"]

# the sensor
sensor_range = 250, math.radians(40)  # range is 250, angle of detection is 40 radians

rbt = Image.open(GUI_DIR + "/gui_images/Nexus_Robot.png")

dt = 0
last_time = pygame.time.get_ticks()

running = True
map_counter = 0
start_condition = True
pygame.init()

obstacleDetected = False
cont = False  # documented in boundary_folllowing_gui.py in avoid_obstacles()
boundary_following = False

# simulation loop
while running:

    # Exit program when close button is clicked
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Check if robot has reached end
    if start_condition:
        if map_counter == len(maps):
            running = False
            break
        start = start_positions[map_counter]
        end = end_positions[map_counter]

        gfx = Graphics(MAP_DIMENSIONS, GUI_DIR + "/gui_images/Nexus_Robot.png",
               GUI_DIR + "/gui_images/" + maps[map_counter] + ".png")

        robot = Robot(start, end, 0.01, 0.01, 0.01, 0.02, 0.005, 100, 5, rbt.size) # robot is ~ 1.5 meters by 1.5 meters

        ultra_sonic = Ultrasonic(sensor_range, gfx.map, None, False)
        ultra_sonic_left_top = Ultrasonic(sensor_range, gfx.map, None, True)
        ultra_sonic_left_bottom = Ultrasonic(sensor_range, gfx.map, None, True)
        ultra_sonic_right_top = Ultrasonic(sensor_range, gfx.map, None, True)
        ultra_sonic_right_bottom = Ultrasonic(sensor_range, gfx.map, None, True)


        map_counter += 1
    start_condition = (robot.x > end[0] and robot.x < end[0] + rbt.size[0]) and (robot.y > end[1] and robot.y < end[1] + rbt.size[1])

    # Add map and get change in time between screen refresh
    dt = (pygame.time.get_ticks() - last_time) / 200
    last_time = pygame.time.get_ticks()
    
    margin_to_obs = 100

    if ultra_sonic.pt is None:
        obst_detected = False
    else:
        obst_detected = ultra_sonic.pt[1] < margin_to_obs
    
    if (not obst_detected) and (not cont) and (not boundary_following):
        # Move robot
        while not robot.updateHeading() == 0:
            last_time = pygame.time.get_ticks()

            robot.stop_moving()
            robot.heading += robot.updateHeading()

            update_viz(gfx, rbt, start, end, robot)
            ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom, obstacleDetected = update_ultrasonics(robot, ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom)

            pygame.display.update()
        robot.move_forward()
        robot.kinematics(dt)
        robot.update_dist()
    else:
        # Execute obstacle avoidance behaviors
        cont, boundary_following = robot.avoid_obstacles(ultra_sonic_right_top.pt, ultra_sonic_right_bottom.pt, ultra_sonic.pt, dt, cont, boundary_following)

        obstacleDetected = False

    # TODO: Add Boolean for if you just did boundary traversal. 
    # If just did boundary traversal, check right side for obstacles in angle to goal
    # should prevent unnecessary turning to target before realizing obstacle and turning back.

    update_viz(gfx, rbt, start, end, robot)
    ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom, obstacleDetected = update_ultrasonics(robot, ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top, ultra_sonic_right_bottom)

    pygame.display.update()
