import json
import math
import numpy as np
from abc import ABC, abstractmethod
from scipy.optimize import minimize

class RobotModel:
    @abstractmethod
    def computeMGD(self, joint):
        """
        Parameters
        ----------
        joints_position : np.array
            The values of the joints of the robot in joint space

        Returns
        -------
        np.array
            The coordinate of the effectors in the operational space
        """

    @abstractmethod
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

    @abstractmethod
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

class RTRobot(RobotModel):
    """
    Model a robot with a 2 degrees of freedom: 1 rotation and 1 translation

    The operational space of the robot is 2 dimensional because it can only move inside a plane
    """
    def __init__(self):
        pass

    def computeMGD(self, joints):
        R1 = np.array([\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [-math.sin(-joints[0]), math.cos(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D1 = np.array([\
            [1, 0, 0, joints[1]+0.2],\
            [0, 1, 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D2 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, -0.25],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        T = R1.dot(D1).dot(D2)
        return [T[0][3],T[1][3]]
        # raise RuntimeError("Not implemented")

    def computeAnalyticalMGI(self, operational_target):
        dist = math.sqrt((operational_target[0])**2 + (operational_target[1])**2)
        max_dist = math.sqrt((0.45)**2 + (0.25)**2)
        min_dist = math.sqrt((0.2)**2 + (0.25)**2)
        if dist < min_dist or dist > max_dist:
            return None, 0

        dYaw = math.atan2(operational_target[1], operational_target[0])
        length = math.sqrt( (dist**2) - (0.25**2));
        p1 = np.array([math.cos(dYaw)*length, math.sin(dYaw)*length])
        alpha= math.atan2(0.25,length)
        return [dYaw+alpha, length-0.2], 1

    def computeJacobian(self, joints):
        raise RuntimeError("Not implemented")

class RRRRobot(RobotModel):
    """
    Model a robot with 3 degrees of freedom along different axis
    """
    def __init__(self):
        pass

    def computeMGD(self, joints):
        return [1,0,0]

    def computeAnalyticalMGI(self, operational_target):
        raise RuntimeError("Not implemented")

    def computeJacobian(self, joints):
        raise RuntimeError("Not implemented")


def searchJacInv(model, joints, target):
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
