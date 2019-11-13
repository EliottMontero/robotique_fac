import math
import random
import numpy as np
import pybullet as p
from time import sleep
from control import updateWheels, autoMovingGoal

debugCount = 0
# Initialisation de pyBullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

# Chargement du robot
cubeStartPos = [0.0, 0, 0.025]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
holo = p.loadURDF("2wheels/robot.urdf", cubeStartPos, cubeStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(holo)

cubeStartPos = [1.5, 0, 0.4]
goal = p.loadURDF("obstacle/goal.urdf", cubeStartPos, cubeStartOrientation)


# Paramètres de vitesse
speedX = p.addUserDebugParameter("speedX", -1, 1, 0)
speedRot = p.addUserDebugParameter("speedRot", -5, 5, 0)

# Contrôle direct des roues
w1 = p.addUserDebugParameter("wheel 1", -5, 5, 0)
w2 = p.addUserDebugParameter("wheel 2", -5, 5, 0)

if not autoMovingGoal:
    goalX = p.addUserDebugParameter("goalX", -5, 5, 2)
    goalY = p.addUserDebugParameter("goalY", -5, 5, 0)

# Joints des roues
wheelIds = [
    p.getJointInfo(holo, 1)[0],
    p.getJointInfo(holo, 2)[0]
]

# Position des roues
wheels = np.array([0., 0.])

# Simulation en temps réel
p.setRealTimeSimulation(0)
t = 0
dt = 0.001
p.setPhysicsEngineParameter(fixedTimeStep=dt)
oldpos = p.getLinkState(holo, 0)[0]

while True:
    # Gestion des obstacles
    obstaclesPos = []
    goalPosition = []

    # Mise à jour de la position cible
    if autoMovingGoal:
        goalPos = np.array([math.cos(2*t*0.1)*1.5, math.sin(2*t*0.1)*1.5])
    else:
        goalPos = np.array([p.readUserDebugParameter(goalX), p.readUserDebugParameter(goalY)])

    p.resetBasePositionAndOrientation(
        goal, [goalPos[0], goalPos[1], 0.03], cubeStartOrientation)

    pos = p.getLinkState(holo, 0)[0]
    orientation = p.getEulerFromQuaternion(p.getLinkState(holo, 0)[1])
    robotYaw = orientation[2]
    robotPos = np.array([pos[0], pos[1]])

    # TRACE

    if np.fmod(debugCount,20) == 0:
        p.addUserDebugLine(pos, oldpos, [0.5, 0.5, 0], 3)
        oldpos = p.getLinkState(holo, 0)[0]
    debugCount += 1


    #FIN TRACE

    # Test si on a atteint la position cible
    dist = np.linalg.norm(robotPos-goalPos)

    if np.fmod(debugCount,1000) == 0:
        if dist < 0.1:
            print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~> Objectif atteint!')
            goalX = p.addUserDebugParameter("goalX", -5, 5, random.uniform(-2,2))
            goalY = p.addUserDebugParameter("goalY", -5, 5, random.uniform(-2,2))

    # Mise à jour des position cibles
    wheels += dt*np.array(updateWheels(t, [p.readUserDebugParameter(speedX),
                                           p.readUserDebugParameter(speedRot)],
                                       robotPos, robotYaw, goalPos))

    wheels += dt*np.array([p.readUserDebugParameter(w1),
                           p.readUserDebugParameter(w2)])

    for w in range(2):
        p.setJointMotorControl2(
            holo, wheelIds[w], p.POSITION_CONTROL, wheels[w])

    # Attente de dt
    sleep(dt)
    t += dt
    p.stepSimulation()
