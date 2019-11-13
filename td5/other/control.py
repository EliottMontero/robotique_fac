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
        raise RuntimeError("Not implemented")

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
        raise RuntimeError("Not implemented")

    def getTarget(self, t):
        raise RuntimeError("Not implemented")

class LinearSplinePose2D(CompositeTrajectory):
    def __init__(self, path):
        raise RuntimeError("Not implemented")

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
    robotYaw = robotPos[2]

    vecto = np.matrix([robotPos[0], robotPos[1]]).T

    matrix = np.array([[math.cos(cartVel[2]),-math.sin(cartVel[2])],[math.sin(cartVel[2]),math.cos(cartVel[2])]])

    end_matrix = matrix.dot(vecto)

    # R = np.array([[math.cos(robotYaw),-math.sin(robotYaw),0],[math.sin(robotYaw),math.cos(robotYaw),0],[0,0,1]])
    # Vxy = np.array([cartVel[0], cartVel[1], 0])
    # Srobot = R.dot(Vxy)

    MKinv = np.array([[math.cos(math.pi/2 + math.pi/3), math.sin(math.pi/2 + math.pi/3), d],\
     [math.cos(math.pi/2 - math.pi/3), math.sin(math.pi/2 - math.pi/3), d],\
     [math.cos(math.pi/2 + math.pi), math.sin(math.pi/2 + math.pi), d]])
    MKfinal = MKinv * (1/r) #MKfinal est la matrice de cinématique inverse
    vect = np.array([end_matrix[0], end_matrix[1], 0]).T #Le vecteur de vitesse transposée

    return Mkfinal.dot(vect)
