import math
import numpy as np
import sys
import time
import pygame
from control import updateRobotPos

# Initialisation de pygame
pygame.init()
size = width, height = 800, 600
screen = pygame.display.set_mode(size)
pygame.display.set_caption('Robot')

# Chargement de l'image du robot
robot = pygame.image.load('robot.png')
robotPos = np.array([100., 0.]).T
mousePos = [0, 0]
screenCenter = np.array([int(width/2), int(height/2)])
myfont = pygame.font.SysFont("monospace", 15)

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    mousePos = np.array(pygame.mouse.get_pos())
    mousePos -= screenCenter
    robotPos = updateRobotPos(robotPos, mousePos)

    # Sol gris
    screen.fill((200, 200, 200))

    # Axes X et Y
    pygame.draw.line(screen, (200, 0, 0),
                     (0, height/2), (width, height/2), 2)
    pygame.draw.line(screen, (0, 200, 0),
                     (width/2, 0), (width/2, height), 2)

    # Texte
    label = myfont.render('X=%f, Y=%f' %
                          tuple(robotPos), 1, (0, 0, 0))
    screen.blit(label, (5, 5))

    # Sprite du robot
    drawImage = pygame.transform.rotozoom(
        robot, -90, 1.)
    robotRect = drawImage.get_rect()
    robotRect.left = robotPos[0] + (width - robotRect.width)/2
    robotRect.top = robotPos[1] + (height - robotRect.height)/2
    screen.blit(drawImage, robotRect)
    pygame.display.flip()
    time.sleep(0.01)
