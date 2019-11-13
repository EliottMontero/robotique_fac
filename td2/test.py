import sys
import numpy as np
import math
import time
import pypot.dynamixel
import pygame

maxSpeed = 75  #vitesse des roues en rpm
speedRatio = 1.339 #ratio pour dxl pour tourner en rpm
rightWheelNb = 1
leftWheelNb = 2
wheelDiameter = 0.0516 # diameter en metre
robotWidth = 0.081 #distance entre roues et milieu robot
sleepTime = 0.01
robotPos = np.array([0., 0.])
robotYaw = 0
dt = 0.02

# Initialisation de pygame
pygame.init()
size = width, height = 800, 600
screen = pygame.display.set_mode(size)
pygame.display.set_caption('Robot')

# Chargement de l'image du robot
robot = pygame.image.load('robot.png')
trace = []
screenCenter = np.array([int(width/2), int(height/2)])
myfont = pygame.font.SysFont("monospace", 15)


ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([2])
dxl_io.set_wheel_mode([1])
dxl_io.set_moving_speed({rightWheelNb:0})
dxl_io.set_moving_speed({leftWheelNb:0})

def diffAngles(angA, angB):
    angA = np.fmod(angA, math.pi)
    angB = np.fmod(angB, math.pi)
    angle = angB - angA
    #if angle > math.pi:
    #    angle -= (math.pi*2)
    #else:
    #    if angle < -math.pi:
    #        angle += (math.pi*2)
    return angle

def wheelSpeedToSpeed(rightWheel, leftWheel):
    robotSpeed = (wheelDiameter*rightWheel + wheelDiameter*leftWheel)/4
    theta = (wheelDiameter*rightWheel - wheelDiameter*leftWheel)/(robotWidth*4)
    return [robotSpeed, theta]

def turnRight(intensity):
    leftSpeed = maxSpeed*intensity*8
    rightSpeed = -maxSpeed
    set_motor_speed(rightSpeed, leftSpeed)
    print("Right turn")

    leftSpeed = rpmToRad(leftSpeed)
    rightSpeed = rpmToRad(rightSpeed)
    return wheelSpeedToSpeed(rightSpeed, leftSpeed)

def set_motor_speed(rightSpeed, leftSpeed):
    dxl_io.set_moving_speed({rightWheelNb:-rightSpeed/speedRatio, leftWheelNb: leftSpeed/speedRatio})

def turnLeft(intensity):
    leftSpeed = -maxSpeed*intensity*8
    rightSpeed = maxSpeed
    set_motor_speed(rightSpeed, leftSpeed)
    print("Left turn")

    leftSpeed = rpmToRad(leftSpeed)
    rightSpeed = rpmToRad(rightSpeed)
    return wheelSpeedToSpeed(rightSpeed, leftSpeed)

def advance():
    leftSpeed = maxSpeed
    rightSpeed = maxSpeed
    set_motor_speed(rightSpeed, leftSpeed)
    print("Advancing")
    leftSpeed = rpmToRad(leftSpeed)
    rightSpeed = rpmToRad(rightSpeed)

    robotSpeed = (wheelDiameter*rightSpeed + wheelDiameter*leftSpeed)/4
    theta = (wheelDiameter*rightSpeed - wheelDiameter*leftSpeed)/(robotWidth*4)
    return [robotSpeed, theta]

def stop():
    dxl_io.set_moving_speed({rightWheelNb:0})
    dxl_io.set_moving_speed({leftWheelNb:0})

def rpmToRad(k):
	return k*math.pi/30

def radToRpm(k):
	return k*30/math.pi

def robotMotion(xDot, thetaDot):
    rightWheel = radToRpm((xDot + thetaDot*robotWidth)/wheelDiameter*0.5)
    leftWheel = radToRpm((xDot - thetaDot*robotWidth)/wheelDiameter*0.5)
    set_motor_speed(rightWheel, leftWheel)
    rightWheel = rpmToRad(rightWheel)
    leftWheel = rpmToRad(leftWheel)
    return wheelSpeedtoSpeed(rightWheel, leftWheel)

def findGoal(goalX, goalY):
    global robotPos
    global robotYaw
    global trace
    if goalX > 0 and goalY == 0:
        goalY += 0.05
    coeffTurn = 0.1
    speed = [0, 0] #Vitesse du robot (avance) et vitesse de rotation
    t = time.time()
    goalPos = np.array([goalX, goalY])
    dist = np.linalg.norm(robotPos-goalPos)
    distAdvance = dist
    while(dist > 0.01):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        # Sol gris
        screen.fill((200, 200, 200))

        # Axes X et Y
        pygame.draw.line(screen, (200, 0, 0),
                         (0, height/2), (width, height/2), 2)
        pygame.draw.line(screen, (0, 200, 0),
                         (width/2, 0), (width/2, height), 2)

        # Texte
        label = myfont.render('X=%f, Y=%f, T=%f' %
                              (robotPos[0], robotPos[1], np.rad2deg(robotYaw)), 1, (0, 0, 0))
        screen.blit(label, (5, 5))

        # Trace du robot
        for k in range(1, len(trace)):
            p1 = trace[k-1]
            p2 = trace[k]
            pygame.draw.line(screen, (200, 0, 200),
                               (p1[0]+width/2., -p1[1]+height/2.),
                               (p2[0]+width/2., -p2[1]+height/2.), 2)

        # Sprite du robot
        drawImage = pygame.transform.rotozoom(
            robot, np.rad2deg(robotYaw)-90, 1.)
        robotRect = drawImage.get_rect()
        robotRect.left = robotPos[0]*100 + (width - robotRect.width)/2
        robotRect.top = -robotPos[1]*100 + (height - robotRect.height)/2
        screen.blit(drawImage, robotRect)
        pygame.display.flip()
        trace += [np.copy(robotPos)]
        trace = trace[-1000:]

        newT = time.time()
        #Ici on met a jour la position des roues par rapport a la vitesse et au timestamp
        robotYaw += speed[1]*dt
        robotPos[0] += np.cos(robotYaw)*speed[0]*dt
        robotPos[1] += np.sin(robotYaw)*speed[0]*dt
        print(robotPos)
        print("robotYaw : ", robotYaw)
        print("goalDiff : ", np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]))
        angularError = diffAngles(robotYaw, np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]))
        print("angularError : ", angularError)
        #On s'oriente vers l'objectif
        if np.abs(angularError) > coeffTurn: #Si on est pas bien orientes vers la cible :
            if angularError < 0:
                speed = turnRight(coeffTurn) #On tourne a droite
            else:
                speed = turnLeft(coeffTurn) #On tourne a gauche
        else:     #On avance vers l'objectif
            speed = advance()
            distAdvance = dist
        dist = np.linalg.norm(robotPos-goalPos)
        t = time.time()
        time.sleep(dt -t + newT) #On dort un peu pour ne pas faire des instructions tout le temps

    print("On est arrives")
    stop()

def goToLinear(goalX, goalY):
    global robotPos
    global robotYaw
    speed = [0, 0] #Vitesse du robot (avance) et vitesse de rotation
    t = time.time()
    goalPos = np.array([goalX, goalY])
    dist = np.linalg.norm(robotPos-goalPos)
    while(dist > 0.05):
        newT = time.time()
        #Ici on met a jour la position des roues par rapport a la vitesse et au timestamp
        robotYaw += speed[1]*dt
        robotPos[0] += np.cos(robotYaw)*speed[0]*dt
        robotPos[1] += np.sin(robotYaw)*speed[0]*dt
        print(robotPos)
        angularError = np.fmod(robotYaw, math.pi) - np.fmod(np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]), math.pi)
        if np.abs(angularError) > 0.3: #Si on est pas bieno orientes vers la cible :
            print(angularError)
            if angularError > 0:
                speed = turnRight(8) #On tourne a droite
            else:
                speed = turnLeft(8) #On tourne a gauche
        else:
            if np.abs(angularError) > 0.1:
                speed = robotMotion(0.8, -angularError*3)
            else:
                speed = advance()
	dist = np.linalg.norm(robotPos-goalPos)
	t = time.time()
        time.sleep(dt -t + newT) #On dort un peu pour ne pas faire des instructions tout le temps

    print("On est arrives")
    stop()

def standby():
    global robotYaw
    global robotPos
    global trace
    dxl_io.disable_torque([leftWheelNb, rightWheelNb])
    while(True):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        # Sol gris
        screen.fill((200, 200, 200))

        # Axes X et Y
        pygame.draw.line(screen, (200, 0, 0),
                         (0, height/2), (width, height/2), 2)
        pygame.draw.line(screen, (0, 200, 0),
                         (width/2, 0), (width/2, height), 2)

        # Texte
        label = myfont.render('X=%f, Y=%f, T=%f' %
                              (robotPos[0], robotPos[1], np.rad2deg(robotYaw)), 1, (0, 0, 0))
        screen.blit(label, (5, 5))

        # Trace du robot
        for k in range(1, len(trace)):
            p1 = trace[k-1]
            p2 = trace[k]
            pygame.draw.line(screen, (200, 0, 200),
                               (p1[0]+width/2., -p1[1]+height/2.),
                               (p2[0]+width/2., -p2[1]+height/2.), 2)

        # Sprite du robot
        drawImage = pygame.transform.rotozoom(
            robot, np.rad2deg(robotYaw)-90, 1.)
        robotRect = drawImage.get_rect()
        robotRect.left = robotPos[0]*100 + (width - robotRect.width)/2
        robotRect.top = -robotPos[1]*100 + (height - robotRect.height)/2
        screen.blit(drawImage, robotRect)
        pygame.display.flip()
        trace += [np.copy(robotPos)]
        trace = trace[-1000:]

    	newT = time.time()
   	rightSpeed = -dxl_io.get_present_speed([rightWheelNb])[0]*(math.pi/30)*speedRatio
    	leftSpeed = dxl_io.get_present_speed([leftWheelNb])[0]*(math.pi/30)*speedRatio
	robotSpeedStandby = (wheelDiameter*rightSpeed + wheelDiameter*leftSpeed)/4
    	theta = (wheelDiameter*rightSpeed - wheelDiameter*leftSpeed)/(robotWidth*4)
    #Ici on met a jour la position des roues par rapport a la vitesse et au timestamp
    	robotYaw += theta*dt
    	robotPos[0] += np.cos(robotYaw)*robotSpeedStandby*dt
    	robotPos[1] += np.sin(robotYaw)*robotSpeedStandby*dt
	print("orientation : ", robotYaw, "pos : ", robotPos)

   	t = time.time()

    	time.sleep(dt - (t - newT))

#goToLinear(float(sys.argv[1]),float(sys.argv[2]))
if sys.argv[1] == "goTo":
    findGoal(float(sys.argv[2]), float(sys.argv[3]))
if sys.argv[1] == "standby":
    standby()
#standby()
