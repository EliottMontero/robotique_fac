import math
import numpy as np

wheelRadius = 9
robotWidth = 27


def avance(t):
    """
    Le robot avance tout droit
    """
    return np.matrix([10, 10]).T


def tourne(t):
    """
    Le robot tourne sur lui meme
    """
    return np.matrix([5, -5]).T


def avanceEtTourne(t):
    """
    Le robot tourne en avancant
    """
    return np.matrix([5, 25]).T


def avanceTourne(t):
    """
    Le robot avance, puis tourne, puis avance, puis tourne etc.
    """
    mod = np.fmod(t, 1)
    if mod < 0.5:
        return avance(t)
    else:
        return tourne(t)


def avanceEtTourneEtAvance(t):
    mod = np.fmod(t, 1)
    if mod < 0.5:
        return avance(t)
    else:
        return avanceEtTourne(t)


def avanceTourneRecul(t):
    """
    Le robot est suppose avancer, puis tourner dans un sens, puis
    tourner dans l'autre sens et reculer
    Il reste normalement statique
    """
    mod = np.fmod(t, 2)
    if mod < 1:
        return avanceTourne(mod)
    else:
        return -avanceTourne(2-mod)


def spirale(t):
    """
    Le robot tourne en decrivant une spirale
    """
    return np.matrix([t, t + 10]).T


def controle(vitesseX, vitesseRot):
    """
    A partir de la vitesse X (en pixels/s) et de rotation (en rad/s),
    cette fonction produit les vitesses des roues correspondantes
    """
    right_speed = (vitesseX + vitesseRot*robotWidth*2)/wheelRadius
    left_speed = (vitesseX - vitesseRot*robotWidth*2)/wheelRadius
    return np.matrix([right_speed, left_speed]).T


def carre(t):
    """
    Le robot suit un carre
    """
    mod = np.fmod(t, 2)
    if mod < 1:
        # On avance pendant 1s a 500px/s
        return controle(200, 0)
    else:
        # On tourne pendant 1s a PI/s
        return controle(0, math.pi/2)


def updateRobotPos(robotPos, mousePos, t, dt):
    # Obtention des vitesses des roues desirees [rad/s]
    wheelSpeeds = carre(t)

    robotSpeed = (wheelRadius*wheelSpeeds[0] + wheelRadius*wheelSpeeds[1])/2
    theta = (wheelRadius*wheelSpeeds[0] - wheelRadius*wheelSpeeds[1])/(robotWidth*4)


    # A vous de jouer !

    # robotPos comporte desormais une troisieme case qui correspond a
    # l'orientation du robot
    # Si on incremente cette orientation a chaque tick, le robot va pivoter
    # sur lui meme !
    robotPos[2] +=  theta*dt
    robotPos[0] += np.cos(robotPos[2])*robotSpeed*dt
    robotPos[1] += np.sin(robotPos[2])*robotSpeed*dt

    return robotPos
