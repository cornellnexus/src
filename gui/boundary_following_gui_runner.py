import math
import time
from PIL import Image

import pygame
from gui.boundary_following_gui import Graphics, Robot, Ultrasonic
from constants.definitions import GUI_DIR

debug = False
MAP_DIMENSIONS = (600, 1200)

# Environment graphics
gfx = Graphics(MAP_DIMENSIONS, GUI_DIR + "/gui_images/Nexus_Robot.png",
               GUI_DIR + "/gui_images/boundary_map.png")
rbt = Image.open(GUI_DIR + "/gui_images/Nexus_Robot.png")

# start and end positions
start = (150, 160)
end = (800, 150)

# the robot
robot = Robot(start, end, 0.01, 0.01, 0.01, 0.02, 0.01, 100, 5)  # robot is ~ 1.5 meters by 1.5 meters

# the sensor
sensor_range = 250, math.radians(40)  # range is 250, angle of detection is 40 radians

sensor_directions = robot.side_sensor_angle()
ultrasonic_lt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
ultrasonic_lb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])
ultrasonic_rt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
ultrasonic_rb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])

ultra_sonic = Ultrasonic(sensor_range, gfx.map, (robot.x, robot.y), False)
ultra_sonic_left_top = Ultrasonic(sensor_range, gfx.map, ultrasonic_lt_loc, True)
ultra_sonic_left_bottom = Ultrasonic(sensor_range, gfx.map, ultrasonic_lb_loc, True)
ultra_sonic_right_top = Ultrasonic(sensor_range, gfx.map, ultrasonic_rt_loc, True)
ultra_sonic_right_bottom = Ultrasonic(sensor_range, gfx.map, ultrasonic_rb_loc, True)

dt = 0
last_time = pygame.time.get_ticks()

dist_to_goal = math.sqrt((end[1] - robot.y) ** 2 + (end[0] - robot.x) ** 2)

running = True
obstacleDetected = False
# simulation loop
while running:

    # Exit program when close button is clicked
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Add map and get change in time between screen refresh
    dt = (pygame.time.get_ticks() - last_time) / 200
    last_time = pygame.time.get_ticks()
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

    # Check if robot has reached end
    if (robot.x > end[0] and robot.x < end[0] + rbt.size[0]) and (robot.y > end[1] and robot.y < end[1] + rbt.size[1]):
        running = False

    # Read sensor data and create a point cloud
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)
    point_cloud_LT = ultra_sonic_left_top.side_sense_obstacles(ultrasonic_lt_loc, robot.heading + math.pi / 2)
    point_cloud_LB = ultra_sonic_left_bottom.side_sense_obstacles(ultrasonic_lb_loc, robot.heading + math.pi / 2)
    point_cloud_RT = ultra_sonic_right_top.side_sense_obstacles(ultrasonic_lb_loc, robot.heading - math.pi / 2)
    point_cloud_RB = ultra_sonic_right_bottom.side_sense_obstacles(ultrasonic_rb_loc, robot.heading - math.pi / 2)
    point_clouds = [point_cloud_LT, point_cloud_LB, point_cloud_RT, point_cloud_RB]

    pt = ultra_sonic.min_distance((robot.x, robot.y), point_cloud)
    pt_LT = ultra_sonic_left_top.min_distance(ultrasonic_lt_loc, point_cloud_LT)
    pt_LB = ultra_sonic_left_bottom.min_distance(ultrasonic_lb_loc, point_cloud_LB)
    pt_RT = ultra_sonic_right_top.min_distance(ultrasonic_rt_loc, point_cloud_RT)
    pt_RB = ultra_sonic_right_bottom.min_distance(ultrasonic_rb_loc, point_cloud_RB)

    ultra_sonic.update_pt(pt)
    ultra_sonic_left_top.update_pt(pt_LT)
    ultra_sonic_left_bottom.update_pt(pt_LB)
    ultra_sonic_right_top.update_pt(pt_RT)
    ultra_sonic_right_bottom.update_pt(pt_RB)

    if robot.detect_obstacles(point_cloud):
        obstacleDetected = True

    new_dist = math.sqrt((end[1] - robot.y) ** 2 + (end[0] - robot.x) ** 2)
    if (not obstacleDetected) and new_dist < dist_to_goal:
        # Move robot
        robot.move_forward()
        robot.kinematics(dt)
        while not robot.updateHeading() == 0:
            robot.stop_moving()
            robot.heading += robot.updateHeading()
            gfx.map.blit(gfx.map_img, (0, 0))
            gfx.draw_robot(robot.x, robot.y, robot.heading)
            pygame.display.update()
            dt = (pygame.time.get_ticks() - last_time) / 200
            last_time = pygame.time.get_ticks()

        dist_to_goal = new_dist
    else:
        # Execute obstacle avoidance behaviors
        robot.avoid_obstacles(point_cloud, point_cloud_LT, point_cloud_LB, point_cloud_RT, point_cloud_RB, dt, pt)
        obstacleDetected = False

    ultrasonics = [ultra_sonic, ultra_sonic_left_top, ultra_sonic_left_bottom, ultra_sonic_right_top,
                   ultra_sonic_right_bottom]
    for ultrasonic in ultrasonics:
        sensor_directions = robot.side_sensor_angle()
        ultrasonic_lt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
        ultrasonic_lb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])
        ultrasonic_rt_loc = (robot.x + sensor_directions[0], robot.y - sensor_directions[1])
        ultrasonic_rb_loc = (robot.x - sensor_directions[0], robot.y + sensor_directions[1])
        ultra_sonic.update_pos((robot.x, robot.y))
        ultra_sonic_left_top.update_pos(ultrasonic_lt_loc)
        ultra_sonic_left_bottom.update_pos(ultrasonic_lb_loc)
        ultra_sonic_right_top.update_pos(ultrasonic_rt_loc)
        ultra_sonic_right_bottom.update_pos(ultrasonic_rb_loc)

    # Draws the robot
    gfx.draw_robot(robot.x, robot.y, robot.heading)

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

    pygame.display.update()
