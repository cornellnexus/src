import pygame
import math
import numpy as np
# Based on https://www.youtube.com/watch?v=pmmUi6DasoM


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, endpos, width):
        self.metersToPixels = 3779.52  # meters to pixels conversion
        # robot dimensions
        self.width = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.endX = endpos[0]
        self.endY = endpos[1]
        self.heading = 0
        self.velocityLeft = 0.01 * self.metersToPixels
        self.velocityRight = 0.01 * self.metersToPixels
        self.maxSpeed = 0.02 * self.metersToPixels
        self.minSpeed = 0.01 * self.metersToPixels
        self.minimumObstacleDistance = 100
        self.countDown = 5  # in seconds

    def avoid_obstacles(self, point_cloud, dt):
        closestObstacle = None
        dist = np.inf
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closestObstacle = (point, dist)
            if closestObstacle[1] < self.minimumObstacleDistance and self.countDown > 0:
                self.countDown -= dt
                self.move_backward()
            else:
                # reset count down
                self.countDown = 5
                # move forward
                self.move_forward()

    def move_backward(self):
        self.velocityRight = - self.minSpeed
        self.velocityLeft = - self.minSpeed/2

    def move_forward(self):
        self.velocityRight = self.minSpeed
        self.velocityLeft = self.minSpeed

    def kinematics(self, dt):
        self.x += ((self.velocityLeft + self.velocityRight) / 2) * \
            math.cos(self.heading) * dt
        self.y -= ((self.velocityLeft + self.velocityRight) / 2) * \
            math.sin(self.heading) * dt
        self.heading += (self.velocityRight -
                         self.velocityLeft) / self.width * dt
        if self.heading > 2*math.pi or self.heading < -2 * math.pi:
            self.heading = 0
        self.velocityRight = max(
            min(self.maxSpeed, self.velocityRight), self.minSpeed)
        self.velocityLeft = max(
            min(self.maxSpeed, self.velocityLeft), self.minSpeed)


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)
        # ---------MAP--------
        # load images
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)
        # dimensions
        self.height, self.width = dimensions
        # window settings
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(
            self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)


class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 10, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles
