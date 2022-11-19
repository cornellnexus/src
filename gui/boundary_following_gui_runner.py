import math

import pygame
from gui.boundary_following_gui import Graphics, Robot, Ultrasonic
from constants.definitions import GUI_DIR

MAP_DIMENSIONS = (600, 1200)

start_positions = [(100, 300), (100, 200), (100, 200), (100, 200)]
end_positions = [(800, 150), (800, 150), (800, 150), (800, 150)]
maps = ["blank", "boundary_map", "roundy_maps", "obstacaly_map"]

# Environment graphics
gfx = Graphics(MAP_DIMENSIONS, GUI_DIR + "/gui_images/Nexus_Robot.png",
               GUI_DIR + "/gui_images/" + maps[0] + ".png")

# start and end positions
start = start_positions[0]
end = end_positions[0]

# the robot
robot = Robot(start, end, 0.01, 0.01, 0.01, 0.02, 0.01, 100, 5)

# the sensor
sensor_range = 250, math.radians(40) # range is 250, angle of detection is 40 radians
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

ultra_sonic_left_top = Ultrasonic(sensor_range, gfx.map)
ultra_sonic_left_bottom = Ultrasonic(sensor_range, gfx.map)
ultra_sonic_right_top = Ultrasonic(sensor_range, gfx.map)
ultra_sonic_right_bottom = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

dist_to_goal = math.sqrt((end[1] - robot.y)**2 + (end[0] - robot.x)**2)

running = True
obstacleDetected = False
# simulation loop
while running:

    # Exit program when close button is clicked
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Check if robot has reached end
    counter = 1
    if (robot.x > end[0] and robot.x < end[0] + 81) and (robot.y > end[1] and robot.y < end[1] + 80):
        start = start_positions[counter]
        end = end_positions[counter]

        gfx = Graphics(MAP_DIMENSIONS, GUI_DIR + "/gui_images/Nexus_Robot.png",
               GUI_DIR + "/gui_images/" + maps[counter] + ".png")

        robot = Robot(start, end, 0.01, 0.01, 0.01, 0.02, 0.01, 100, 5)

        ultra_sonic = Ultrasonic(sensor_range, gfx.map)
        ultra_sonic_left_top = Ultrasonic(sensor_range, gfx.map)
        ultra_sonic_left_bottom = Ultrasonic(sensor_range, gfx.map)
        ultra_sonic_right_top = Ultrasonic(sensor_range, gfx.map)
        ultra_sonic_right_bottom = Ultrasonic(sensor_range, gfx.map)

        dist_to_goal = math.sqrt((end[1] - robot.y)**2 + (end[0] - robot.x)**2)

        counter += 1

    # Add map and get change in time between screen refresh
    dt = (pygame.time.get_ticks() - last_time) / 500
    last_time = pygame.time.get_ticks()
    gfx.map.blit(gfx.map_img, (0, 0))

    # Adding start surface (blue box)
    startSurf = pygame.Surface((81, 80))
    startSurf.fill(gfx.blue)
    gfx.map.blit(startSurf, (start[0] - 40, start[1] - 40))

    # Adding end surface (red box)
    endSurf = pygame.Surface((81, 80))
    endSurf.fill(gfx.red)
    gfx.map.blit(endSurf, (end[0], end[1]))

    # Adding green following line
    pygame.draw.line(gfx.map, gfx.green, start, (end[0] + 40, end[1] + 41))

    # Read sensor data and create a point cloud
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)

    sensor_directions = robot.side_sensor_angle()
    point_cloud_LT = ultra_sonic_left_top.side_sense_obstacles(robot.x + sensor_directions[0], robot.y - sensor_directions[1], robot.heading + math.pi/2)
    point_cloud_LB = ultra_sonic_left_bottom.side_sense_obstacles(robot.x - sensor_directions[0], robot.y + sensor_directions[1], robot.heading + math.pi/2)
    point_cloud_RT = ultra_sonic_right_top.side_sense_obstacles(robot.x + sensor_directions[0], robot.y - sensor_directions[1], robot.heading - math.pi/2)
    point_cloud_RB = ultra_sonic_right_bottom.side_sense_obstacles(robot.x - sensor_directions[0], robot.y + sensor_directions[1], robot.heading - math.pi/2)
    
    pt = ultra_sonic.min_distance((robot.x, robot.y), point_cloud)
    
    pt_LT = ultra_sonic_left_top.min_distance((robot.x + sensor_directions[0], robot.y - sensor_directions[1]), point_cloud_LT)
    pt_LB = ultra_sonic_left_bottom.min_distance((robot.x - sensor_directions[0], robot.y + sensor_directions[1]), point_cloud_LB)
    pt_RT = ultra_sonic_right_top.min_distance((robot.x + sensor_directions[0], robot.y - sensor_directions[1]), point_cloud_RT)
    pt_RB = ultra_sonic_right_bottom.min_distance((robot.x - sensor_directions[0], robot.y + sensor_directions[1]), point_cloud_RB)
    point_clouds = [point_cloud_LT, point_cloud_LB, point_cloud_RT, point_cloud_RB]

    if robot.detect_obstacles(point_cloud):
        obstacleDetected = True

    new_dist = math.sqrt((end[1] - robot.y)**2 + (end[0] - robot.x)**2)
    if (not obstacleDetected) and new_dist < dist_to_goal:
        # Move robot
        robot.move_forward()
        while not robot.updateHeading() == 0:
            robot.heading += robot.updateHeading()
        robot.kinematics(dt)
        dist_to_goal = new_dist
    else:
        # Execute obstacle avoidance behaviors
        robot.avoid_obstacles(point_cloud, point_cloud_LT, point_cloud_LB, point_cloud_RT, point_cloud_RB, dt, pt)
        obstacleDetected = False

    # Draws the robot
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    # Draw point cloud and update screen
    gfx.draw_sensor_data(point_cloud)
    gfx.draw_side_sensor_data(point_cloud_LT)
    gfx.draw_side_sensor_data(point_cloud_LB)
    gfx.draw_side_sensor_data(point_cloud_RT)
    gfx.draw_side_sensor_data(point_cloud_RB)

    pygame.display.update()
