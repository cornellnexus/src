import time
import pygame
import math
import numpy as np
# Based on https://www.youtube.com/watch?v=pmmUi6DasoM


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, endpos, width, velocityLeft, velocityRight, maxSpeed, minSpeed, minimumObstacleDistance, countDown):
        self.metersToPixels = 3779.52  # meters to pixels conversion
        # robot dimensions
        self.width = width * self.metersToPixels
        self.x = startpos[0]
        self.y = startpos[1]
        self.startX = startpos[0]
        self.startY = startpos[1]
        self.endX = endpos[0]
        self.endY = endpos[1]
        self.setHeading()
        self.velocityLeft = velocityLeft * self.metersToPixels
        self.velocityRight = velocityRight * self.metersToPixels
        self.maxSpeed = maxSpeed * self.metersToPixels
        self.minSpeed = minSpeed * self.metersToPixels
        self.minimumObstacleDistance = minimumObstacleDistance
        self.countDown = countDown  # in seconds
        self.closestObstacle = None
        self.distFromClosestObstacle = np.inf

    def detect_obstacles(self, point_cloud):
        if len(point_cloud) > 0:
            for point in point_cloud:
                if self.distFromClosestObstacle > distance([self.x, self.y], point):
                    self.distFromClosestObstacle = distance([self.x, self.y], point)
                    self.closestObstacle = point
            if self.distFromClosestObstacle < self.minimumObstacleDistance:
                return True
        self.closestObstacle = None
        self.distFromClosestObstacle = np.inf
        return False

    def avoid_obstacles(self, front_point_cloud, left_top_point_cloud, left_bottom_point_cloud, right_top_point_cloud, right_bottom_point_cloud, dt, min_front):
        margin_to_obs = 75
        #left_value = self.check_parallel(left_top_point_cloud, left_bottom_point_cloud, 1)
        parallel = self.check_parallel(right_top_point_cloud, right_bottom_point_cloud, 15)
        # if left_value == 0 and right_value == 0:
        if min_front == None:
            condition = False
        else:
            condition = min_front[1] < margin_to_obs
        if condition:
            self.heading += 0.003
        else:
            if parallel == 0:
                # distance_to_front_obstacle = np.inf
                # for point in front_point_cloud:
                #     distance_to_front_obstacle = min(distance(point, (self.x, self.y)), distance_to_front_obstacle)
                self.move_forward()
                self.kinematics(dt)
            else:
                #self.heading += -0.001 * left_value + -0.001 * right_value
                # while not self.check_parallel(right_top_point_cloud, right_bottom_point_cloud, 1.5) == 0:
                self.heading += -0.005 * parallel

    def check_parallel(self, fpc, bpc, margin):
        min_dist_front = np.inf
        min_dist_back = np.inf
        for point in fpc:
            local_min_distance = min(distance(point, (self.x, self.y)), self.minimumObstacleDistance)
            min_dist_front = min(min_dist_front, local_min_distance)
        for point in bpc:
            local_min_distance = min(distance(point, (self.x, self.y)), self.minimumObstacleDistance)
            min_dist_back = min(min_dist_back, local_min_distance)
        if abs(min_dist_front - min_dist_back) < margin:
            return 0
        if min_dist_front > min_dist_back:
            return 1
        return -1

    def stop_moving(self):
        self.velocityRight = 0
        self.velocityLeft = 0

    def move_backward(self):
        self.velocityRight = - self.minSpeed
        self.velocityLeft = - self.minSpeed/2

    def move_forward(self):
        self.velocityRight = self.minSpeed
        self.velocityLeft = self.minSpeed

    def arctan(self, startX, startY, endX, endY):
        xLength = endX - startX
        yLength = endY - startY
        return np.arctan(yLength / xLength)

    def setHeading(self):
        # Angle between robot and line to destination
        angle = self.arctan(self.startX, self.startY, self.endX + 40, self.endY + 41)
        self.heading = angle * -1

    def updateHeading(self):
        angle = self.arctan(self.x, self.y, self.endX + 40, self.endY + 41)
        curr_angle = self.heading
        while curr_angle > 2*math.pi:
            curr_angle = curr_angle - 2*math.pi
        while curr_angle < 0:
            curr_angle = curr_angle + 2*math.pi
        finalHeading = -angle
        while finalHeading > 2*math.pi:
            finalHeading = finalHeading - 2*math.pi
        while finalHeading < 0:
            finalHeading = finalHeading + 2*math.pi
        if np.abs(finalHeading - curr_angle) > 0.1:
            if finalHeading > curr_angle:
                return .01
            else:
                return -.01
        else:
            return 0

    def kinematics(self, dt):
        self.x += ((self.velocityLeft + self.velocityRight) / 2) * \
            math.cos(self.heading) * dt
        self.y -= ((self.velocityLeft + self.velocityRight) / 2) * \
            math.sin(self.heading) * dt
        self.velocityRight = min(self.maxSpeed, self.velocityRight)
        self.velocityLeft = min(self.maxSpeed, self.velocityLeft)

    def side_sensor_angle(self):
        x = 20 * math.cos(self.heading)
        y = 20 * math.sin(self.heading)
        return [x, y]


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        # Colors
        self.black = (0.1, 0.1, 0.1)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)
        self.purple = (255, 0, 255)
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

    def draw_side_sensor_data(self, point_clouds):
        count = 0
        for point_cloud in point_clouds:
            for point in point_cloud:
                if count % 3 == 0:
                    pygame.draw.circle(self.map, self.purple, point, 3, 0)
                count = count + 1

    def draw_pt(self, pt):
        pygame.draw.circle(self.map, self.blue, pt[0], 10, 0)

    def draw_side_pt_clouds(self, side_pt_clouds):
        for pt_cloud in side_pt_clouds:
            self.draw_side_sensor_data(pt_cloud)

class Ultrasonic:
    def __init__(self, sensor_range, map, pos, is_side):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map
        self.pos = pos
        self.pt = None
        self.is_side = is_side

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
    
    def side_sense_obstacles(self, pos, heading):
        obstacles = []
        x = pos[0]
        y = pos[1]
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
                    #self.map.set_at((x, y), (75, 0, 130))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles

    def min_distance(self, curr_pos, point_cloud):
        if len(point_cloud) == 0:
            return None
        else:
            min_dist = math.dist(point_cloud[0], curr_pos)
            min_point = point_cloud[0]
            for points in point_cloud:
                dist = math.dist(points, curr_pos)
                if dist < min_dist:
                    min_dist = dist
                    min_point = points
            return (min_point, min_dist)

    def update_pt(self, pt):
        self.pt = pt

    def update_pos(self, pos):
        self.pos = pos