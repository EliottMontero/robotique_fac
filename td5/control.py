import json
import math
import numpy as np
from numpy.linalg import inv
from abc import ABC, abstractmethod
import time

speedRatio = 30
r = 0.0325 #rayon des roues en mètres
d = 0.08 #distance entre la roue et le centre du robot

class BangBang:
    """
    BangBang control law on acceleration along a single dimension
    """
    def __init__(self, src, dst, startT, maxSpeed, maxAcc):
        """startT is used to specify the starting time of the trajectory"""
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class PIDController:
    def __init__(self, kp, ki, kd, maxCartOrder, maxThetaOrder):
        """
        Parameters
        ----------
        kp : float
        ki : float
        kd : float
        maxCartOrder : float
            The maximal cartesian speed
        maxThetaOrder : float
            The maximal angular speed
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maxCartOrder = maxCartOrder
        self.maxThetaOrder = maxThetaOrder
        #raise RuntimeError("Not implemented")

    def step(self, t, pos, target, feedForward = np.array([0,0,0])):
        """
        Parameters
        ----------
        t : float
            time [s] used for integration
        pos : np.array[3]
            The current position of the robot
        target : np.array[3]
            The target of the robot
        feedForward : np.array[3]
            The initially expected order

        Returns
        -------
        np.array[3]
           The resulting order
        """
        #err donne la difference entre les pos target et robot en np.array[3]
        err = np.array([target[0]-pos[0], target[1]-pos[1], target[2]-pos[2]]) * self.kp
        #dist la distance cartesienne entre target et robot
        dist = np.linalg.norm(np.array([pos[0], pos[1]])- np.array([target[0], target[1]]))

        #On dit qu'arrive a 0.1 m de la target, on est arrives et on s'oriente
        #par rapport a l'orientation voulue si ce n'est pas encore parfait
        if dist < 0.1:
            if math.fabs(math.fmod(pos[2], math.pi*2)-math.fmod(target[2], math.pi*2)) > 0.1:
                if math.fmod(pos[2], math.pi*2)-math.fmod(target[2], math.pi*2) > 0:
                    err[2] = -self.maxThetaOrder/2
                else:
                    err[2] = self.maxThetaOrder/2
            else:
                    return np.array([0, 0, 0])

        #on verifie si on ne va pas trop vite
        if math.fabs(err[0]) > self.maxCartOrder:
            err[0] = self.maxCartOrder*np.sign(err[0])
        if math.fabs(err[1]) > self.maxCartOrder:
            err[1] = self.maxCartOrder*np.sign(err[1])

        #on verifie si le feedForward va trop vite, et si on peut le suivre
        #sans depasser la vitesse max
        if math.fabs(err[0]) < math.fabs(feedForward[0]):
            err[0] = min(math.fabs(feedForward[0]), math.fabs(self.maxCartOrder))
            err[0] = err[0]*np.sign(feedForward[0])

        if math.fabs(err[1]) < math.fabs(feedForward[1]):
            err[1] = min(math.fabs(feedForward[1]), math.fabs(self.maxCartOrder))
            err[1] = err[1]*np.sign(feedForward[1])


        return np.array([err[0], err[1], err[2]])

class Trajectory(ABC):

    @abstractmethod
    def getTarget(self, t):
        """
        Parameters
        ----------
        t : float
            Actual time [s]

        Returns
        -------
        targetPos : np.array[3]
            The target position (x[m],y[m],theta[rad])
        targetVel : np.array[3]
            The target velocity (vx[m/s], vy[m/s], vTheta [rad/s])
        """

    def getTargetPos(self, t):
        return self.getTarget(t)[0]

    def getTargetVel(self, t):
        return self.getTarget(t)[1]

class CompositeTrajectory(Trajectory):
    def __init__(self):
        self.trajectories = []

    def getTarget(self, t):
        targetPos = []
        targetVel = []
        for i in range(3):
            p, v = self.trajectories[i].getTarget(t)
            targetPos.append(p)
            targetVel.append(v)
        return [np.array(targetPos),np.array(targetVel)]


class FixedTarget(Trajectory):
    def __init__(self, order):
        """
        Parameters
        ----------

        order : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential
        """
        self.order = order

    def getTarget(self,t):
        return self.order, np.array([0,0,0])

class LinearTarget(Trajectory):
    def __init__(self, speed):
        """
        Parameters
        ----------
        speed : np.array[3]
              vx [m/s], vy[m/s], vTheta [rad/s] wished speed in world referential
        """
        self.speed = speed

    def getTarget(self,t):
        return t * self.speed, self.speed

class BangBangPose2D(CompositeTrajectory):
    def __init__(self, src, dst, start_t, maxSpeed, maxAcc, maxThetaSpeed, maxThetaAcc):
        """
        Parameters
        ----------
        src : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential at start
        dst : np.array[3]
              x [m], y[m], theta [rad] position of the robot in world referential at end
        """
        raise RuntimeError("Not implemented")

class BangBangWheel(CompositeTrajectory):
    def __init__(self, src, dst, start_t, maxWheelSpeed, maxWheelAcc):
        raise RuntimeError("Not implemented")

class HoloSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class DirectSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class LinearSpline:
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """

        self.knots = knots

    def getTarget(self, t):
        #la classe qui va faire la trajectoire, renvoie la position actuelle
        #et la vitesse souhaitée
        knots = self.knots
        next_t = knots[0][0]
        for i in range(len(knots)-1):
            if t > knots[i][0] and t < knots[i+1][0]:
                break
        #on est donc avec knots[i]<t<knots[i+1]
        #il faut effectuer diff mouvement pour aller a la position souhaitee
        diff = knots[i+1][1]-knots[i][1]
        #il reste diffTime avant la prochaine target, celle ou on a fini diff
        diffTime = knots[i+1][0]-knots[i][0]
        #pour faire diff mouvement en diffTime temps, il faut aller a diff/diffTime vitesse
        speed = diff/diffTime

        #d'apres le temps et les trajectoires precedentes, on doit se trouver a
        #pos au temps t
        pos = knots[i][1] + speed*diffTime
        return (pos, speed)

class LinearSplinePose2D(CompositeTrajectory):
    def __init__(self, path):

        #On ouvre le json
        f = open(path)
        data = f.read()
        datastore = json.loads(data)
        #initialisation des listes ou on va mettre les donnees
        posX = [None]*len(datastore['x'])
        t = [None]*len(datastore['x'])
        posY = [None]*len(datastore['y'])
        posT = [None]*len(datastore['theta'])
        k=0
        #on recupere les donnees
        for d in datastore['x']:
            posX[k] = float(d[1])
            t[k] = float(d[0])
            k = k+1
        k=0
        for d in datastore['y']:
            posY[k] = float(d[1])
            k = k+1
        k=0
        for d in datastore['theta']:
            posT[k] = float(d[1])
            k = k+1

        #on initialise les listes pour y mettre les points de controles par dimension
        knotsX = [[[None], [None]] for _ in range(len(datastore['x']))]
        knotsY = [[[None], [None]] for _ in range(len(datastore['y']))]
        knotsT = [[[None], [None]] for _ in range(len(datastore['theta']))]
        for i in range(len(posX)):
            knotsX[i][0]= t[i]
            knotsX[i][1] =posX[i]
        for i in range(len(posY)):
            knotsY[i][0]= t[i]
            knotsY[i][1] =posY[i]
        for i in range(len(posT)):
            knotsT[i][0]= t[i]
            knotsT[i][1] =posT[i]

        #on met une spline par dimension dans self.trajectories
        self.trajectories = [LinearSpline(knotsX), LinearSpline(knotsY), LinearSpline(knotsT)]

class CubicSpline:
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class CubicSplinePose2D(CompositeTrajectory):
    def __init__(self, path):
        raise RuntimeError("Not implemented")

def cartVelToWheelVel(robotPos, cartVel):
    """
    Parameters
    ----------
    robotPos : np.array[3]
        pose of the robot in world referential (x[m], y[m], theta [rad])
    cartVel : np.array[3]
        target velocity [vx,vy,vTheta] in world referential
    Returns
    -------
    np.array[3]
        wheelVelocities [w1,w2,w3]
    """
    #

    robotYaw = robotPos[2]

    R = np.array([[math.cos(robotYaw), -math.sin(robotYaw)], [math.sin(robotYaw), math.cos(robotYaw)]]).transpose()

    localVel = R.dot(np.array([[cartVel[0]], [cartVel[1]]]));


    #VERSION BON SENS
    MKinv = np.array([[math.cos(-math.pi/3 - math.pi/2), math.sin(-math.pi/3 - math.pi/2), -d],\
     [math.cos(math.pi/3 - math.pi/2), math.sin(math.pi/3 - math.pi/2), -d],\
     [math.cos(math.pi/2), math.sin(math.pi/2), -d]]) /r

    #VERSION SENS INVERSE
    # MKinv = np.array([[math.cos(math.pi/3 + math.pi/2), math.sin(math.pi/3 + math.pi/2), -d],\
    #  [math.cos(-math.pi/3 + math.pi/2), math.sin(-math.pi/3 + math.pi/2), -d],\
    #  [math.cos(-math.pi/2), math.sin(-math.pi/2), -d]]) /r


    vect = np.array([localVel[0], localVel[1], np.fmod(cartVel[2], math.pi*2)]).T #Le vecteur de vitesse transposée

    MKfinal = MKinv.dot(vect)
    wheelVel = np.array([MKfinal[0], MKfinal[1], MKfinal[2]])

    return wheelVel


if __name__ == "__main__":
    robotPos = [0.0, 0.0, 0.0]
    cartVel = [0.2, 0.0, 1.0]
    print(cartVelToWheelVel(robotPos, cartVel))
