import numpy as np
import math

# Mouvement automatique de l'objectif
autoMovingGoal = False
utiliseCarre = False #Booléen pour faire le carré
utiliseCarreHolo = False #Booléen pour faire le carré Holo
suiviTurn = False #Booléen pour passer en mode suivi avec orientation
suiviDirect = True #Booléen pour passer en mode suivi direct, sans orientation
r = 0.0325 #rayon des roues en mètres
d = 0.08 #distance entre la roue et le centre du robot
dist_min_goal = 0.005 #La distance max entre nous et l'objectif
speedRatio = 30 #Ce ratio me permet d'avoir une translation et une rotation en accord avec le temps
#Je dois multiplier mes vitesses par ce ratio, sinon elles ne sont pas en accord avec le temps

def carre(t): #A 0,5 de vitesse on a un bon carré, car pas trop de problème de drift

    mod = np.fmod(t, 2)
    if mod < 1:
        return [0.5,0,0]
    else:
        return [0,0,math.pi/2]



def carreHolo(t): #Je met une vitesse de 0.25 car quand on va trop vite ça a tendance à
                  #drifter et on perd en précision vu que le robot avance suivant son propre repère

    mod = np.fmod(t, 4)
    if mod < 1:
        return [0.25,0,0]
    else:
        if mod < 2:
            return [0, 0.25, 0]
        else:
            if mod < 3:
                return [-0.25, 0, 0]
            else:
                return [0, -0.25, 0]


def speedToWheelSpeed(speed): #Prend en argument un array de speed [vx, vy, theta], et renvoie
                              #un array de wheel speed [w1, w2, w3]

    speed[1] = -speed[1] #Quand on veut se déplacer dans le Y, ça va dans le mauvais sens
                         #par rapport à l'axe, on change donc le sens du Y
    MKinv = np.array([[math.cos(math.pi/2 + math.pi/3), math.sin(math.pi/2 + math.pi/3), d],\
     [math.cos(math.pi/2 - math.pi/3), math.sin(math.pi/2 - math.pi/3), d],\
     [math.cos(math.pi/2 + math.pi), math.sin(math.pi/2 + math.pi), d]])
    MKfinal = MKinv * (1/r) #MKfinal est la matrice de cinématique inverse
    vect = np.array([speed[0]*speedRatio, speed[1]*speedRatio, speed[2]*speedRatio]).T #Le vecteur de vitesse transposée

    return MKinv.dot(vect)





def updateWheels(t, speed, robotPos, robotYaw, goalPos):
    """Appelé par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        speed {float[3]} -- les vitesses entrées dans les curseurs [m/s], [m/s], [rad/s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde

    Returns:
        float[3] -- les vitesses des roues [rad/s]
    """

    #Ici on voit si on veut tester le mode carré ou carréHolo, ils sont activés si
    #leurs booléens respectifs sont à True, le carréHolo ne peut fonctionner si le carré
    #est à True, seul le carré fonctionnera dans ce cas.
    #Les fonctions carré et carréHolo change l'array speed[] pour changer
    #le comportement du robot.
    if not suiviTurn and not suiviDirect: #En mode basique, pas de suivi
        if utiliseCarre: #En mode fait des carrés
            speed = carre(t)
        else:
            if utiliseCarreHolo: #En mode fait des carrés Holo
                speed=carreHolo(t)

        finalK = speedToWheelSpeed(speed)
        return [finalK[0], finalK[1], finalK[2]]

    else:
        if suiviTurn: #En mode suivi d'objectif en tournant, ne peut faire carré ou carréHolo
            #On s'oriente d'abord vers l'objectif, avec une erreur angulaire max de 0.15
            if np.abs(np.fmod(robotYaw, math.pi) - np.fmod(np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]), math.pi)) > 0.15:
                if np.fmod(robotYaw, math.pi) - np.fmod(np.arctan2(goalPos[1]-robotPos[1], goalPos[0]-robotPos[0]), math.pi) > 0:
                    finalK = speedToWheelSpeed([0,0,math.pi/4])
                    return [finalK[0], finalK[1], finalK[2]]
                else:
                    finalK = speedToWheelSpeed([0,0,-math.pi/4])
                    return [finalK[0], finalK[1], finalK[2]]
            finalK = speedToWheelSpeed([0.5,0,0]) #Ici on a la bonne orientation, on va donc avancer à une vitesse de 0.5 sur l'axe des x (devant donc)
            if np.abs(np.abs(robotPos[0])-np.abs(goalPos[0])) > dist_min_goal:
                return [finalK[0], finalK[1], finalK[2]]
            if np.abs(np.abs(robotPos[1])-np.abs(goalPos[1])) > dist_min_goal:
                return [finalK[0], finalK[1], finalK[2]]
            return [0,0,0]

        else: #En mode suivi d'objectif direct, ne peut faire carré ou carréHolo
            #Ici on prend le ration vitesse X / vitesse Y pour arriver à l'objectif
            toX = goalPos[0]-robotPos[0]
            toY = goalPos[1]-robotPos[1]

            #Si on est très éloignés, ça risque d'être beaucoup trop rapide et de tout casser,
            #Donc on réduit le ratio jusqu'à atteindre une vitesse max de 0.5 (La même utilisée
            #pour le carré)
            while np.abs(toX) > 0.5 or np.abs(toY) > 0.5:
                toX = toX/1.25
                toY = toY/1.25
            #On transforme la vitesse en vitesse de roues
            finalK = speedToWheelSpeed([toX, toY, 0])
            #Et c'est bon, plus qu'à avancer si on est assez loin (Je n'ai pas calculé la distance
            # exacte pour éviter d'utiliser trop d'appels à math, sqrt, etc)
            if np.abs(np.abs(robotPos[0])-np.abs(goalPos[0])) > dist_min_goal:
                return [finalK[0], finalK[1], finalK[2]]
            if np.abs(np.abs(robotPos[1])-np.abs(goalPos[1])) > dist_min_goal:
                return [finalK[0], finalK[1], finalK[2]]
            return [0,0,0]
