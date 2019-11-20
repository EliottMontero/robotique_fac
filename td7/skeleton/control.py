import json
import math
import numpy as np
from abc import ABC, abstractmethod
from scipy.optimize import minimize

def normAngle(a):
    """
    Return 'a' normalized in [-pi,pi[
    """
    a = a % (2*np.pi)
    if a >= np.pi:
        a -= 2*np.pi
    return a

def adjustedAngleTarget(a,b):
    """
    return the value b+2*k*pi the closest to a
    """
    if abs(b-a) > np.pi:
        signDiff = 1 if b > a else -1
        b -= signDiff * 2*np.pi
    return b

class LegModel:
    def __init__(self):
        pass

    def computeMGD(self, joints):
        """
        Parameters
        ----------
        joints_position : np.array
            The values of the joints of the robot in joint space

        Returns
        -------
        np.array
            The coordinate of the effector in the operational space
        """
        return np.array([0,0,0,0])

    def computeAnalyticalMGI(self, operational_target):
        """
        Parameters
        ----------
        operational_target : np.array
            The target given in the operational space

        Returns
        -------
        np.array or None
            The target for the robot in joints space
        int
            The number of solutions
        """
        return None, 0

    def computeJacobian(self, joints):
        """
        Parameters
        ----------
        joints : np.array
            Current position of all the joints

        Returns
        -------
        np.array
            The jacobian of the robot
        """
        raise RuntimeError("Not implemented")

def searchJacInv(model, joints, target, depth= 0, max_depth= 10, max_eps = 0.1):
    """
    Parameters
    ----------
    model : RobotModel
        The model of the robot used to compute MGD and Jacobian
    joints : np.array
        Initial position of the joints
    target : np.array
        The target in operational space

    Returns
    -------
    np.array
        The wished position for joints in order to reach the target
    """
    raise RuntimeError("Not implemented")

def searchJacTransposed(model, joints, target):
    """
    Parameters
    ----------
    model : RobotModel
        The model of the robot used to compute MGD and Jacobian
    joints : np.array
        Initial position of the joints
    target : np.array
        The target in operational space

    Returns
    -------
    np.array
        The wished position for joints in order to reach the target
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
        targetPos : np.array[4]
            Either in operational space or angular space
        targetVel : np.array[4]
            Either in operational space or angular space
        """

    @abstractmethod
    def getDuration(self):
        """
        Returns:
        float : the duration of the trajectory [s]
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
        for traj in self.trajectories:
            p, v = traj.getTarget(t)
            targetPos.append(p)
            targetVel.append(v)
        return [np.array(targetPos),np.array(targetVel)]

    def getDuration(self):
        d = 0
        for traj in self.trajectories:
            d = max(traj.getDuration(), d)
        return d


class LinearSpline:
    def __init__(self, knots, is_angle = False):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """
        self.knots = knots
        self.is_angle = is_angle

    def getTarget(self, t):
        if len(self.knots) == 0:
            raise RuntimeError("Getting target in an empty spline")
        # Extremum cases
        pos = 0
        vel = 0
        if self.knots[0][0] > t:
            pos = self.knots[0][1]
        elif self.knots[-1][0] < t:
            pos = self.knots[-1][1]
        for i in range(len(self.knots) - 1):
            nextT = self.knots[i+1][0]
            if nextT < t:
                continue
            srcT = self.knots[i][0]
            srcPos = self.knots[i][1]
            nextPos = self.knots[i+1][1]
            if self.is_angle:
                nextPos = adjustedAngleTarget(srcPos, nextPos)
            vel = (nextPos - srcPos) / (nextT -srcT)
            pos = srcPos + vel * (t-srcT)
            if self.is_angle:
                pos = normAngle(pos)
            break
        return pos, vel

    def getDuration(self):
        return self.knots[-1][0]

class LinearSplinePose(CompositeTrajectory):
    def __init__(self, path):
        splines = {}
        with open(path) as splineFile:
            splines = json.load(splineFile)
        self.trajectories = []
        for key in ["x", "y", "z","r32"]:
            self.trajectories.append(LinearSpline(splines[key], key == "theta"))

class CubicSpline:
    def __init__(self, knots, is_angle = False):
        """
        Parameters
        ----------
        knots : list((float,float,float))
            The list of knots (time, position, derivative)

        """
        self.knots = knots
        self.is_angle = is_angle

    def getDuration(self):
        return self.knots[-1][0]

    def getTarget(self, t):
        if len(self.knots) == 0:
            raise RuntimeError("Getting target in an empty spline")
        pos = 0
        vel = 0
        if self.knots[0][0] > t:
            pos = self.knots[0][1]
        elif self.knots[-1][0] < t:
            pos = self.knots[-1][1]
        for i in range(len(self.knots) - 1):
            nextT = self.knots[i+1][0]
            if nextT < t:
                continue
            srcT = self.knots[i][0]
            dt = nextT - srcT
            srcPos = self.knots[i][1]
            nextPos = self.knots[i+1][1]
            if self.is_angle:
                nextPos = adjustedAngleTarget(srcPos, nextPos)
            srcDer = self.knots[i][2]
            nextDer = self.knots[i+1][2]
            # Normalize time diff in [0,1]
            x = (t-srcT) / dt
            srcDer *= dt
            nextDer *= dt
            # Compute polynomial A*x^3 + B*x^2 + C*x + D
            # 1. f(0)  : srcPos = D
            # 2. f'(0) : srcDer = C
            # 3. f(1)  : nextPos = A + B + C + D
            # 4. f'(1) : nextDer = 3A + 2B + C
            # From 3: A = nextPos - B - C - D
            # From 3 and 4:
            # * nextDer = 3*nextPos - B -2*C -3*D
            # * B = 3*nextPos - nextDer - 2*C - 3*D
            D = srcPos
            C = srcDer
            B =  3* nextPos - nextDer - 2*C - 3*D
            A = nextPos - B - C - D
            pos = A * x**3 + B * x**2 + C*x + D
            vel = 3 * A * x**2 + 2*B*x + C
            # Normalizing vel since we substitued x = t /dt
            vel /= dt
            # print ("t,x,A,B,C,D,pos,vel: {:3f},{:3f},{:3f},{:3f},{:3f},{:3f},{:3f},{:3f}".format(t,x,A,B,C,D,pos,vel))
            break
        if self.is_angle:
            pos = normAngle(pos)
        return pos, vel

class CubicSplinePose(CompositeTrajectory):
    def __init__(self, path):
        splines = {}
        with open(path) as splineFile:
            splines = json.load(splineFile)
        self.trajectories = []
        for key in ["x", "y", "z","r32"]:
            self.trajectories.append(CubicSpline(splines[key], key == "theta"))
