import pygame
import math
import numpy as np


# Based on https://www.youtube.com/watch?v=pmmUi6DasoM

# TODO: fix issue where when the robot turns to avoid wall in sharp angle, 
#        sometimes "bounces back". Issues appears in 
#        "fixed problem with robot getting stuck" commit. Likely have something 
#        to do with self.gate. Try not to revert change because it did help with
#        bug with not catching the robot when it became parallel to obstacle 
#        after turning to avoid wall/obstacle
# TODO: more refactoring and renaming would be good
# TODO: add documentation to actual documentation document
# TODO: double check updateHeading; I don't think it turns in the shortest 
#        direction to goal

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)

class Robot:
    def __init__(self, startpos, endpos, width, velocityLeft, velocityRight, maxSpeed, minSpeed,
                 minimumObstacleDistance, countDown):
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
        self.was_parallel = False
        self.gate = True

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

    def avoid_obstacles(self, pt_rt, pt_rb, dt, min_front, cont):
        margin_to_obs = 100
        min_sensor_val = 120
        side_sensor_margin = 3

        # initializaing values of the ultrasonic sensors
        if pt_rt is None:
            right_front = min_sensor_val
        else:
            right_front = min(pt_rt[1], min_sensor_val)
        if pt_rb is None:
            right_back = min_sensor_val
        else:
            right_back = min(pt_rb[1], min_sensor_val)
        if min_front is None:
            # obst_detected can be false because this is in reference to a second obstacle, like a sharp turn
            obst_detected = False  
        else:
            obst_detected = min_front[1] < margin_to_obs

        # finds out whether the robot is parallel to obst or not
        # made was_parallel and gate attributes of robot bc persistent and 
        # I dont want to the sketchy accumulator thing with cont again
        if abs(right_front - right_back) < side_sensor_margin:
            parallel = 0
            self.was_parallel = True
        elif right_front > right_back:
            parallel = 1
            self.was_parallel = False
        else:
            parallel = -1
            self.was_parallel = False
        
        # cont determines whether to continue turning to finish clearing obstacle. 
        # This prevents the robot from getting stuck between turning to clear obstacle and maintaining parallel to obst
        # stops turning to clear obstacle (cont becomes false) when 
        # the robot becomes parallel to the wall: when the robot turns and fully reset against obs
        if obst_detected:
            cont = True
        # gate is there in case the robot was starting parallel and then turning to avoid obstacle. 
        # If gate didn't exist, then if the robot was initially parallel,
        # then cont will immediately become false because the robot is already parallel to obst,
        # so the program thinks that the robot already finish turning to clear obstacle
        # this is when the condition is <cont and not parallel == 0> instead of
        #  <cont and (self.gate or (not self.gate and not parallel == 0))>
        if cont and (self.gate or (not self.gate and not parallel == 0)): # assume margin to obs large enough that turning won't hit the side
            self.heading += 0.02
            if not self.was_parallel:
                self.gate = False
            return True
        else:
            cont = False
            self.gate = True
        
        # second obstacle not detected
        if not cont:
            if parallel == 0:
                self.move_forward()
                self.kinematics(dt)
            else:
                self.heading += -0.01 * parallel
            return False

    def stop_moving(self):
        self.velocityRight = 0
        self.velocityLeft = 0

    def move_forward(self):
        self.velocityRight = self.minSpeed
        self.velocityLeft = self.minSpeed

    def setHeading(self):
        # Angle between robot and line to destination
        angle = math.atan2(self.endY + 41 - self.y, self.endX + 40 - self.startX)
        self.heading = angle * -1

    def updateHeading(self):
        angle = math.atan2(self.endY + 41 - self.y, self.endX + 40 - self.x)
        curr_angle = self.heading
        while curr_angle > 2 * math.pi:
            curr_angle = curr_angle - 2 * math.pi
        while curr_angle < 0:
            curr_angle = curr_angle + 2 * math.pi
        finalHeading = -angle
        while finalHeading > 2 * math.pi:
            finalHeading = finalHeading - 2 * math.pi
        while finalHeading < 0:
            finalHeading = finalHeading + 2 * math.pi
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
                    # self.map.set_at((x, y), (75, 0, 130))
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
