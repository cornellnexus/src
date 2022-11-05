import math
import pygame
from gui.boundary_following_gui import Graphics, Robot, Ultrasonic
from constants.definitions import GUI_DIR

MAP_DIMENSIONS = (600, 1200)

# Environment graphics
gfx = Graphics(MAP_DIMENSIONS, GUI_DIR+"/gui_images/Aditya Robot.png",
               GUI_DIR+"/gui_images/boundary_map.png")

# start and end positions
start = (150, 160)
end = (800, 150)

# the robot
robot = Robot(start, end, 0.01 * 3779.52)

# the sensor
sensor_range = 250, math.radians(40) # range is 240, angle of detection is 40 radians
ultra_sonic = Ultrasonic(sensor_range, gfx.map)
dt = 0
last_time = pygame.time.get_ticks()

running = True
# simulation loop
while running:

    # Exit program when close button is clicked
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Add map and get change in time between screen refresh
    dt = (pygame.time.get_ticks() - last_time) / 1000
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

    # Check if robot has reached end
    if (robot.x > end[0] and robot.x < end[0] + 81) and (robot.y > end[1] and robot.y < end[1] + 80):
        running = False

    # Move robot
    robot.kinematics(dt)

    # Draws the robot
    gfx.draw_robot(robot.x, robot.y, robot.heading)

    # Read sensor data and create a point cloud
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)

    # Execute obstacle avoidance behavior
    robot.avoid_obstacles(point_cloud, dt)

    # Draw point cloud and update screen
    gfx.draw_sensor_data(point_cloud)
    pygame.display.update()
