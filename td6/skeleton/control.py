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
        R11 = np.array([\
            [math.sin(-joints[0]), -math.cos(-joints[0]), 0, 0],\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        R12 = np.array([\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [-math.sin(-joints[0]), math.cos(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D1 = np.array([\
            [1, 0, 0, 0.2],\
            [0, 1, 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D21 = np.array([\
            [1, 0, 0, joints[1]],\
            [0, 1, 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D22 = np.array([\
            [0, 0, 0, 1],\
            [0, 0, 0, 0],\
            [0, 0, 0, 0],\
            [0, 0, 0, 0]])
        D3 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, -0.25],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])

        T1 = R11.dot(D1).dot(D21).dot(D3)
        T2 = R12.dot(D1).dot(D22).dot(D3)




        jacobian = np.array([[T1[0][3], T2[0][3]],\
        [T1[1][3], T2[1][3]]])
        return jacobian

class RRRRobot(RobotModel):
    """
    Model a robot with 3 degrees of freedom along different axis
    """
    def __init__(self):
        pass

    def computeMGD(self, joints):
        L1 = 0.4
        L2 = 0.3
        L3 = 0.31
        H = 1.01
        D0 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, 0],\
            [0, 0, 1, H],\
            [0, 0, 0, 1]])
        R1 = np.array([\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [-math.sin(-joints[0]), math.cos(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        R2 = np.array([\
            [1, 0 , 0, 0],\
            [0, math.cos(-joints[1]), math.sin(-joints[1]), 0],\
            [0, -math.sin(-joints[1]), math.cos(-joints[1]), 0],\
            [0, 0, 0, 1]])
        R3 = np.array([\
            [1, 0 , 0, 0],\
            [0, math.cos(-joints[2]), math.sin(-joints[2]), 0],\
            [0, -math.sin(-joints[2]), math.cos(-joints[2]), 0],\
            [0, 0, 0, 1]])
        D1 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L1],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D2 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L2],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D3 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L3],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])

        T = D0.dot(R1).dot(D1).dot(R2).dot(D2).dot(R3).dot(D3)
        result = np.array([T[0][3], T[1][3], T[2][3]])
        return result

    def computeAnalyticalMGI(self, operational_target):
        raise RunTimeError("Not implemented")

    def computeJacobian(self, joints):
        L1 = 0.4
        L2 = 0.3
        L3 = 0.31
        H = 1.01
        D0 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, 0],\
            [0, 0, 1, H],\
            [0, 0, 0, 1]])
        R1 = np.array([\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [-math.sin(-joints[0]), math.cos(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        R1dv = np.array([\
            [math.sin(-joints[0]), -math.cos(-joints[0]), 0, 0],\
            [math.cos(-joints[0]), math.sin(-joints[0]), 0, 0],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        R2 = np.array([\
            [1, 0 , 0, 0],\
            [0, math.cos(-joints[1]), math.sin(-joints[1]), 0],\
            [0, -math.sin(-joints[1]), math.cos(-joints[1]), 0],\
            [0, 0, 0, 1]])
        R2dv = np.array([\
            [1, 0 , 0, 0],\
            [0, math.sin(-joints[1]), -math.cos(-joints[1]), 0],\
            [0, math.cos(-joints[1]), math.sin(-joints[1]), 0],\
            [0, 0, 0, 1]])
        R3 = np.array([\
            [1, 0 , 0, 0],\
            [0, math.cos(-joints[2]), math.sin(-joints[2]), 0],\
            [0, -math.sin(-joints[2]), math.cos(-joints[2]), 0],\
            [0, 0, 0, 1]])
        R3dv = np.array([\
            [1, 0 , 0, 0],\
            [0, math.sin(-joints[2]), -math.cos(-joints[2]), 0],\
            [0, math.cos(-joints[2]), math.sin(-joints[2]), 0],\
            [0, 0, 0, 1]])
        D1 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L1],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D2 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L2],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])
        D3 = np.array([\
            [1, 0, 0, 0],\
            [0, 1, 0, L3],\
            [0, 0, 1, 0],\
            [0, 0, 0, 1]])


        T1 = D0.dot(R1dv).dot(D1).dot(R2).dot(D2).dot(R3).dot(D3)
        T2 = D0.dot(R1).dot(D1).dot(R2dv).dot(D2).dot(R3).dot(D3)
        T3 = D0.dot(R1).dot(D1).dot(R2).dot(D2).dot(R3dv).dot(D3)

        jacobian = np.array([[T1[0][3], T2[0][3], T3[0][3]],\
        [T1[1][3], T2[1][3], T3[1][3]],\
        [T1[2][3], T2[2][3], T3[2][3]]])
        return jacobian



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
    inv_jacobian = np.linalg.inv(model.computeJacobian(joints))
    G = model.computeMGD(joints)
    result = inv_jacobian.dot(target-G)
    return result+joints

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
