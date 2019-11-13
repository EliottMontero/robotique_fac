import numpy as np
import math

# Mouvement automatique de l'objectif
autoMovingGoal = False
robotWidth = 0.1
wheelRadius = 0.025
dist_min_goal = 0.05
autoMovingGoal = False

CompteurRandom = 0


def updateWheels(t, speed, robotPos, robotYaw, goalPos):
    """Appelé par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        speed {float[2]} -- les vitesses entrées dans les curseurs [m/s], [rad/s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde

    Returns:
        float[2] -- les vitesses des roues [rad/s]
    """
    #right_speed = (speed[0] + speed[1]*robotWidth)/wheelRadius
    #left_speed = (speed[0] - speed[1]*robotWidth)/wheelRadius
    right_speed = 0
    left_speed = 0
    max_speed = 20

    ### VERSION IDIOTE RIGOLOTE #####
    # if np.abs(robotPos[0]-goalPos[0]) > 0.02:
    #     if np.abs(robotYaw) > 0.1:
    #         return[max_speed,-max_speed]
    #     else:
    #         if robotPos[0] < goalPos[0]:
    #             return[max_speed,max_speed]
    #         else:
    #             return[-max_speed,-max_speed]
    # if np.abs(robotPos[1]-goalPos[1]) > 0.02:
    #     if np.abs(robotYaw-math.pi/2) > 0.1:
    #         return[max_speed,-max_speed]
    #     else:
    #         if robotPos[1] < goalPos[1]:
    #             return[max_speed,max_speed]
    #         else:
    #             return[-max_speed,-max_speed]

    #Parfois le robot va aller à l'opposé de la ou il doit aller
    #Si l'obstacle ne bouge pas, il n'y arrivera jamais
    #Donc on change l'orientation du robot de maniere periodique
    #Pour eviter de ne jamais arriver a destination



    if np.abs(np.fmod(robotYaw, math.pi) - np.fmod(np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]), math.pi)) > 0.1:
        if np.fmod(robotYaw, math.pi) - np.fmod(np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]), math.pi) > 0:
            return[max_speed/4,max_speed]
        else:
            return(max_speed,max_speed/4)
    if np.abs(np.abs(robotPos[0])-np.abs(goalPos[0])) > dist_min_goal:
        return[max_speed,max_speed]
    if np.abs(np.abs(robotPos[1])-np.abs(goalPos[1])) > dist_min_goal:
        return[max_speed,max_speed]

    return [right_speed, left_speed]
