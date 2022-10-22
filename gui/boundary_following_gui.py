import pygame
import math

WIDTH, HEIGHT = 600, 1200


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)
        self.height, self.width = dimensions
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(
            self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))


gfx = Graphics((WIDTH, HEIGHT),
               "boundary_following_robot.png", "boundary_map.png")


def main():
    gfx = Graphics((WIDTH, HEIGHT),
                   "boundary_following_robot.png", "boundary_map.png")
    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            gfx.map.blit(gfx.map_img, (0, 0))
        pygame.display.update()
    pygame.quit()


if __name__ == "__boundary_following_gui__":
    main()
