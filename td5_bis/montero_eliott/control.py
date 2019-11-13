import json
import math
import numpy as np
from abc import ABC, abstractmethod


r = 0.0325 #rayon des roues en mètres
d = 0.08 #distance entre la roue et le centre du robot

class BangBang:
    """
    BangBang control law on acceleration along a single dimension
    """
    def __init__(self, src, dst, startT, maxSpeed, maxAcc):
        """startT is used to specify the starting time of the trajectory"""
        self.src = src
        self.dst = dst
        self.startT = startT
        self.maxSpeed = maxSpeed
        self.maxAcc = maxAcc
        self.lastT = startT
        self.speed = 0
        self.pos = src
        self.decelerating = False

    def getTarget(self, t):
        if t >= self.startT:
            if self.pos != self.dst:
                dist_pos_to_dst = self.dst - self.pos
                dt = t - self.lastT
                self.lastT = t
                dist_to_stop = (self.speed**2)/(2*self.maxAcc)

                if self.decelerating:
                    self.speed = self.speed - (self.maxAcc * np.sign(dist_pos_to_dst) *dt)
                else:
                    self.speed = self.speed + (self.maxAcc * np.sign(dist_pos_to_dst) *dt)

                if np.sign(self.speed) != np.sign(dist_pos_to_dst):
                    self.speed = -self.speed
                if math.fabs(self.speed) > self.maxSpeed:
                    self.speed = self.maxSpeed * np.sign(self.speed)
                if math.fabs(dist_pos_to_dst) <= dist_to_stop:
                    self.decelerating = True

                self.pos = self.pos + self.speed*dt
                print(self.pos, self.speed)
                return(self.pos, self.speed)
            return(self.pos, self.speed)
        return(self.src, self.speed)

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
        errX = target[0]-pos[0] +feedForward[0]
        errY = target[1]-pos[1] +feedForward[1]
        errTheta = math.atan2(math.sin(target[2]-pos[2]+feedForward[2]), math.cos(target[2]-pos[2]+feedForward[2]))

        #print("target : ", target, "pos :", pos, "errortheta :", target[2]-pos[2])
        err = np.array([errX, errY, errTheta]) * self.kp

        if math.fabs(err[0]) > self.maxCartOrder:
            err[0] = self.maxCartOrder*np.sign(err[0])
        if math.fabs(err[1]) > self.maxCartOrder:
            err[1] = self.maxCartOrder*np.sign(err[1])
        if math.fabs(err[2]) > self.maxThetaOrder:
            err[2] = self.maxThetaOrder*np.sign(err[2])

        return err


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
        self.trajectories = [BangBang(src[0], dst[0], start_t, maxSpeed, maxAcc)\
        , BangBang(src[1], dst[1], start_t, maxSpeed, maxAcc)\
        , BangBang(src[2], dst[2], start_t, maxThetaSpeed, maxThetaAcc)]

class BangBangWheel(CompositeTrajectory):
    def __init__(self, src, dst, start_t, maxWheelSpeed, maxWheelAcc):
        maxSpeed = maxWheelSpeed * math.pi * 2 * r
        maxAcc = maxWheelAcc * math.pi * 2 * r
        maxThetaSpeed = (r * maxWheelSpeed) / d
        maxThetaAcc = (r * maxWheelAcc) / d
        self.trajectories = [BangBang(src[0], dst[0], start_t, maxSpeed, maxAcc)\
        , BangBang(src[1], dst[1], start_t, maxSpeed, maxAcc)\
        , BangBang(src[2], dst[2], start_t, maxThetaSpeed, maxThetaAcc)]

class HoloSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        self.size = size
        self.maxSpeed = maxSpeed
        self.maxAcc = maxAcc
        self.maxThetaSpeed = maxThetaSpeed
        self.maxThetaAcc = maxThetaAcc
        self.timeToNext = size*3 #Le temps entre deux trajectoires
        self.trajectories1 = [BangBang(0, size, 0, maxSpeed, maxAcc)\
        , BangBang(0, 0, 0, maxSpeed, maxAcc)\
        , BangBang(0,0,0, maxSpeed,maxAcc)]
        self.trajectories2 = [BangBang(size, size, self.timeToNext, maxSpeed, maxAcc)\
        , BangBang(0, size, self.timeToNext, maxSpeed, maxAcc)\
        , BangBang(0,0,self.timeToNext, maxSpeed,maxAcc)]
        self.trajectories3 = [BangBang(size, 0, self.timeToNext*2, maxSpeed, maxAcc)\
        , BangBang(size, size, self.timeToNext*2, maxSpeed, maxAcc)\
        , BangBang(0,0,self.timeToNext*2, maxSpeed,maxAcc)]
        self.trajectories4 = [BangBang(0, 0, self.timeToNext*3, maxSpeed, maxAcc)\
        , BangBang(size, 0, self.timeToNext*3, maxSpeed, maxAcc)\
        , (BangBang(0,0,self.timeToNext*3, maxSpeed,maxAcc))]

    def getTarget(self, t):
        targetPos = []
        targetVel = []

        t = np.fmod(t, self.timeToNext*4)
        if t < self.timeToNext:
            for i in range(3):
                p, v = self.trajectories1[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*2:
            for i in range(3):
                p, v = self.trajectories2[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*3:
            for i in range(3):
                p, v = self.trajectories3[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t <= self.timeToNext*4:
            for i in range(3):
                p, v = self.trajectories4[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        return [np.array(targetPos),np.array(targetVel)]

class DirectSquare(Trajectory):
    def __init__(self, size, maxSpeed,maxAcc, maxThetaSpeed, maxThetaAcc):
        self.size = size
        self.maxSpeed = maxSpeed
        self.maxAcc = maxAcc
        self.maxThetaSpeed = maxThetaSpeed
        self.maxThetaAcc = maxThetaAcc
        self.timeToNext = size*3 #Le temps entre deux trajectoires
        self.trajectories1 = [BangBang(0, size, 0, maxSpeed, maxAcc)\
        , BangBang(0, 0, 0, maxSpeed, maxAcc)\
        , BangBang(0,0,0, maxThetaSpeed, maxThetaAcc)]
        self.trajectories2 = [BangBang(0, 0, self.timeToNext, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext, maxSpeed, maxAcc)\
        , BangBang(0,math.pi/2,self.timeToNext, maxThetaSpeed, maxThetaAcc)]
        self.trajectories3 = [BangBang(0, size, self.timeToNext*1.5, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*1.5, maxSpeed, maxAcc)\
        , BangBang(0,0,self.timeToNext*1.5, maxThetaSpeed, maxThetaAcc)]
        self.trajectories4 = [BangBang(0, 0, self.timeToNext*2.5, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*2.5, maxSpeed, maxAcc)\
        , BangBang(0,math.pi/2,self.timeToNext*2.5, maxThetaSpeed, maxThetaAcc)]
        self.trajectories5 = [BangBang(0, size, self.timeToNext*3, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*3, maxSpeed, maxAcc)\
        , BangBang(0,0,self.timeToNext*3, maxThetaSpeed, maxThetaAcc)]
        self.trajectories6 = [BangBang(0, 0, self.timeToNext*4, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*4, maxSpeed, maxAcc)\
        , BangBang(0,math.pi/2,self.timeToNext*4, maxThetaSpeed, maxThetaAcc)]
        self.trajectories7 = [BangBang(0, size, self.timeToNext*4.5, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*4.5, maxSpeed, maxAcc)\
        , BangBang(0,0,self.timeToNext*4.5, maxThetaSpeed, maxThetaAcc)]
        self.trajectories8 = [BangBang(0, 0, self.timeToNext*5.5, maxSpeed, maxAcc)\
        , BangBang(0, 0, self.timeToNext*5.5, maxSpeed, maxAcc)\
        , BangBang(0,math.pi/2,self.timeToNext*5.5, maxThetaSpeed, maxThetaAcc)]

    def getTarget(self, t):
        targetPos = []
        targetVel = []

        t = np.fmod(t, self.timeToNext*1.5*4)
        if t < self.timeToNext:
            for i in range(3):
                p, v = self.trajectories1[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*1.5:
            for i in range(3):
                p, v = self.trajectories2[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*2.5:
            for i in range(3):
                p, v = self.trajectories3[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*3:
            for i in range(3):
                p, v = self.trajectories4[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*4:
            for i in range(3):
                p, v = self.trajectories5[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*4.5:
            for i in range(3):
                p, v = self.trajectories6[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t < self.timeToNext*5.5:
            for i in range(3):
                p, v = self.trajectories7[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        elif t <= self.timeToNext*6:
            for i in range(3):
                p, v = self.trajectories8[i].getTarget(t)
                targetPos.append(p)
                targetVel.append(v)
        return [np.array(targetPos),np.array(targetVel)]

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
        #il faut effectuer diff mouvement pour aller a la position souhaitee depuis
        #la precedente
        diff = knots[i+1][1]-knots[i][1]
        #il reste diffTime avant la prochaine target depuis la precedente
        diffTime = knots[i+1][0]-knots[i][0]
        #pour faire diff mouvement en diffTime temps, il faut aller a diff/diffTime vitesse
        speed = diff/diffTime

        #d'apres le temps et les trajectoires precedentes, on doit se trouver a
        #pos au temps t
        pos = knots[i][1] + speed*(t-knots[i][0])
        if t>knots[i+1][0]:
            return(knots[i+1][1], 0)
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
        knots : list((float,float,float))
            The list of knots (time, position, derivative)
        """

        self.knots = knots

    def getTarget(self, t):
        knots = self.knots
        next_t = knots[0][0]
        for i in range(len(knots)-1):
            if t >= knots[i][0] and t < knots[i+1][0]:
                break
        #on est donc avec knots[i]<t<knots[i+1]
        x1 = knots[i][0]
        x2 = knots[i+1][0]
        y1 = knots[i][1]
        y1_2 = knots[i+1][1]
        y2 = knots[i][2]
        y2_2 = knots[i+1][2]

        eqX = np.array([[x1**3, x1**2, x1, 1], [x2**3, x2**2, x2, 1],\
         [3*(x1**2), 2*x1, 1, 0], [3*(x2**2), 2*x2, 1, 0]])
        eqY = np.array([y1, y1_2, y2, y2_2])

        res = np.linalg.solve(eqX, eqY)
        a = res[0]
        b = res[1]
        c = res[2]
        d = res[3]

        pos = a*(t**3) + b*(t**2) +c*(t) +d
        speed = 3*a*(t**2)+2*b*t + c
        if t>knots[i+1][0]:
            return(knots[i+1][1], 0)
        return (pos, speed)

class CubicSplinePose2D(CompositeTrajectory):
    def __init__(self, path):
        #On ouvre le json
        f = open(path)
        data = f.read()
        datastore = json.loads(data)
        #initialisation des listes ou on va mettre les donnees
        posX = [None]*len(datastore['x'])
        dvX = [None]*len(datastore['x'])
        t = [None]*len(datastore['x'])
        posY = [None]*len(datastore['y'])
        dvY = [None]*len(datastore['y'])
        posT = [None]*len(datastore['theta'])
        dvT = [None]*len(datastore['theta'])
        k=0
        #on recupere les donnees
        for d in datastore['x']:
            dvX[k] = float(d[2])
            posX[k] = float(d[1])
            t[k] = float(d[0])
            k = k+1
        k=0
        for d in datastore['y']:
            dvY[k] = float(d[2])
            posY[k] = float(d[1])
            k = k+1
        k=0
        for d in datastore['theta']:
            dvT[k] = float(d[2])
            posT[k] = float(d[1])
            k = k+1

        #on initialise les listes pour y mettre les points de controles par dimension
        knotsX = [[[None], [None], [None]] for _ in range(len(datastore['x']))]
        knotsY = [[[None], [None], [None]] for _ in range(len(datastore['y']))]
        knotsT = [[[None], [None], [None]] for _ in range(len(datastore['theta']))]
        for i in range(len(posX)):
            knotsX[i][0] = t[i]
            knotsX[i][1] = posX[i]
            knotsX[i][2] = dvX[i]
        for i in range(len(posY)):
            knotsY[i][0] = t[i]
            knotsY[i][1] = posY[i]
            knotsY[i][2] = dvY[i]
        for i in range(len(posT)):
            knotsT[i][0] = t[i]
            knotsT[i][1] = posT[i]
            knotsT[i][2] = dvT[i]

        #on met une spline par dimension dans self.trajectories
        self.trajectories = [CubicSpline(knotsX), CubicSpline(knotsY), CubicSpline(knotsT)]

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

    robotYaw = robotPos[2]

    R = np.array([[math.cos(robotYaw), -math.sin(robotYaw)], [math.sin(robotYaw), math.cos(robotYaw)]]).transpose()

    localVel = R.dot(np.array([[cartVel[0]], [cartVel[1]]]));

    MKinv = np.array([[math.cos(-math.pi/3 - math.pi/2), math.sin(-math.pi/3 - math.pi/2), -d],\
     [math.cos(math.pi/3 - math.pi/2), math.sin(math.pi/3 - math.pi/2), -d],\
     [math.cos(math.pi/2), math.sin(math.pi/2), -d]]) /r

    vect = np.array([localVel[0], localVel[1], np.fmod(cartVel[2], math.pi*2)])

    MKfinal = MKinv.dot(vect)
    wheelVel = np.array([MKfinal[0], MKfinal[1], MKfinal[2]])

    return wheelVel
