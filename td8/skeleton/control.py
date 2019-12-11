import json
import math
import numpy as np
from abc import ABC, abstractmethod
from scipy.optimize import minimize

from homogeneous_transform import *

defaultParameters = {
    "restX" : 0.0,
    "restY" : 0.19,
    "restZ" : -0.08,
    "flyingRatio" : 0.25,
    "stepHeight" : 0.02
}

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

def cosineLaw(a,b,c):
    """
    Returns:
    --------
    float or None
        angle [rad] opposite to side of length 'c' in a triangle with sides 'a', 'b' and 'c'
        or None on failures
    """
    numerator = a**2 + b**2 - c**2
    denominator = 2 * a * b
    result = numerator / denominator
    if (abs(result) > 1.0):
        return None
    result = numerator / denominator
    return math.acos(result)

class LegModel:
    """
    Model a leg from the center of the robot, considering 'y' axis goes toward
    the first joint of the leg
    """
    def __init__(self):
        self.L0 =  55*10**-3
        self.L1 =  42*10**-3
        self.L2 =  63*10**-3
        self.L3 =  71*10**-3
        self.L4 = 105*10**-3
        self.T_0_1 = translation([0,-self.L0,0])
        self.T_1_2 = translation([0,-self.L1,0])
        self.T_2_3 = translation([0,-self.L2,0])
        self.T_3_4 = translation([0,-self.L3,0])
        self.T_4_E = translation([0,-self.L4,0])

    def getJointConfigScore(self, joints):
        score = 0
        if (joints[1] < 0):
            score = score -100
        if (joints[2] > 0):
            score = score -10
        if (abs(joints[1]) > np.pi):
            score = score -1
        return score

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
        # Definining readable target variables
        X = operational_target[0]
        Y = operational_target[1]
        zDir = operational_target[3]# zDir: norm of vector y_Tool in frame 0
        solutions = []
        # Two options for q0 (only axis one with impact on rotation)
        alpha = 0
        if np.linalg.norm(operational_target[:2]) > 10**-10:
            alpha = math.atan2(Y,X) - np.pi/2
        for q0 in [alpha, alpha + np.pi]:
            # From now on, we can solve all elements as if it was only on a
            # plane after first articulation.q0
            T_1_0 = rotZ(q0).dot(invertTransform(self.T_0_1))
            # Getting target in this referential
            target_in_1 = T_1_0.dot(np.concatenate([operational_target[:3],[1]]))
            # sin(q1+q2+q3) = planeDir
            beta = math.asin(zDir)
            for q123 in [beta, np.pi - beta]:
                CE = rotX(-q123).dot(self.T_4_E).dot(np.array([0,0,0,1]))
                # Now we have triangle ABC, with:
                # A: pos(q1)
                # B: pos(q2)
                # C: pos(q3)
                A = self.T_1_2.dot(np.array([0,0,0,1]))
                C = target_in_1 - CE
                AC = C[:3] - A[:3]
                LX = np.linalg.norm(AC)
                # print("q0, q123, LX: {:}, {:}, {:}".format(q0,q123, LX))
                # print("A: {:}".format(A))
                # print("C: {:}".format(C))
                # print("CE: {:}".format(CE))
                # print("Target_in_1: {:}".format(target_in_1))
                if LX > self.L2 + self.L3:
                    continue
                q1_offset = math.atan2(AC[2], AC[1])# orientation of AC
                # print("q1_offset: {:}".format(q1_offset))
                A_angle = cosineLaw(self.L2, LX, self.L3)
                B_angle = cosineLaw(self.L2, self.L3, LX)
                if A_angle is None or B_angle is None:
                    continue
                for sign in [-1,1]:
                    q1 = sign * A_angle + q1_offset
                    q2 = sign * (B_angle - np.pi)
                    q3 = q123 - q1 - q2
                    solutions.append(np.array([q0,q1,q2,q3]))
        if len(solutions) == 0:
            return None, 0
        best_sol = solutions[0]
        best_score = self.getJointConfigScore(solutions[0])
        for i in range(1,len(solutions)):
            score = self.getJointConfigScore(solutions[i])
            if score > best_score:
                best_sol = solutions[i]
                best_score = score
        return best_sol, len(solutions)

class QuadrupedModel:
    def __init__(self):
        self.leg_model = LegModel()
        self.trunk_from_leg = []
        for i in range(4):
            self.trunk_from_leg.append(rotZ(np.pi/4 - i*np.pi/2))

    def analyticalMGI(self, leg_targets, current_joint_pos):
        """
        Parameters
        ----------
        leg_targets: np.array(3,4)
            each column is the cartesian target of leg in robot referential
        """
        joints = np.copy(current_joint_pos)
        for i in range(4):
            leg_from_trunk = invertTransform(self.trunk_from_leg[i])
            target = leg_from_trunk.dot(np.concatenate((leg_targets[:,i], [1])))
            target[3] = -1 # Forcing orientation of the leg
            leg_joints, nb_sols = self.leg_model.computeAnalyticalMGI(target)
            if nb_sols > 0:
                joints[4*i:4*(i+1)] = leg_joints
        return joints

class WalkEngine:
    """
    restingPose: np.array(3)
        Default position for a leg in cartesian space
    """
    def __init__(self, robot_model):
        self.robot_model = robot_model

    def getPeriod(self):
        return 1.0 / self.speed_factor

    def getAffix(self, t):
        period = self.getPeriod()
        return (t % period) / period

    def getLegAffix(self, t, leg_id):
        period = self.getPeriod()
        if leg_id == 1:
            t += period / 2
        if leg_id == 2:
            t += period / 4
        if leg_id == 3:
            t += 3 * period / 4
        return (t % period) / period

    def setParameters(self,parameters):
        self.speed_factor = parameters["speed"]
        self.restingPose = np.array([
            parameters["restX"],
            parameters["restY"],
            parameters["restZ"]
        ])

    @abstractmethod
    def getLegTargets(self,t):
        """
        Parameters
        ----------
        t: float
            Time since walk was started

        Returns
        -------
        leg_targets: np.array(3,4)
            The targets for each leg at given time
        """

class FixedWalkEngine(WalkEngine):
    def __init__(self, robot_model):
        super().__init__(robot_model)

    def getLegTargets(self,t):
        leg_targets = np.zeros((3,4))
        for legId in range(4):
            resting_pos = self.robot_model.trunk_from_leg[legId].dot(np.concatenate((self.restingPose,[1])))[:3]
            leg_targets[:,legId] = resting_pos
        return leg_targets


class TrajectoryWalkEngine(WalkEngine):
    def __init__(self, trajectory, robot_model):
        super().__init__(robot_model)
        self.trajectory = trajectory
    """
    Simply addition the default trajectory with restingPosition
    """
    def getPeriod(self):
        return self.trajectory.getDuration() / self.speed_factor

    def getLegTargets(self, t):
        leg_targets = np.zeros((3,4))
        for i in range(4):
            # Getting resting pos for leg in robot referential
            resting_pos = self.robot_model.trunk_from_leg[i].dot(np.concatenate((self.restingPose,[1])))[:3]
            leg_time = self.getLegAffix(t, i) * self.getPeriod() * self.speed_factor
            offset = self.trajectory.getTarget(leg_time)[0]
            leg_targets[:,i] = resting_pos + offset
        return leg_targets

class ParametricWalkEngine(WalkEngine):
    """
    stepHeight : float
        Height of the steps in [m]
    flyingRatio : float
        Ratio of time spent in the air for each leg
    """
    def __init__(self, robot_model):
        super().__init__(robot_model)

    def setParameters(self,parameters):
        super().setParameters(parameters)
        self.stepX = parameters["stepX"]
        self.stepY = parameters["stepY"]
        self.stepDir = parameters["stepDir"]
        self.stepHeight = parameters["stepHeight"]
        self.flyingRatio = parameters["flyingRatio"]

    def getLegHeight(self, affix):
        if( affix < self.flyingRatio and affix > 0):
            return self.stepHeight * (affix/self.flyingRatio)
        return 0

    def getLegX(self, affix):
        if( affix < self.flyingRatio):
            return self.stepX * (affix/self.flyingRatio)
        if( affix >= self.flyingRatio):
            return self.stepX - ((2*self.stepX) * (affix-self.flyingRatio) * (1/(1-self.flyingRatio)))
        return 0

    def getLegY(self, affix):
        if( affix < self.flyingRatio and affix > 0):
            return self.stepY * (affix/self.flyingRatio)
        if( affix >= self.flyingRatio):
            return self.stepY - ((2*self.stepY) * (affix-self.flyingRatio) * (1/(1-self.flyingRatio)))
        return 0

    def getLegDir(self, affix):
        rotStep = self.stepDir/(math.pi*10) #pi*10 = ratio pour que ça ne parte pas trop vite
        if( affix < self.flyingRatio and affix > 0):
            return rotStep * (affix/self.flyingRatio)
        if( affix >= self.flyingRatio):
            return rotStep - ((2*rotStep) * (affix-self.flyingRatio) * (1/(1-self.flyingRatio)))
        return 0


class LegLifter(ParametricWalkEngine):
    """
    Simply lift the legs
    """
    def getLegTargets(self, t):
        leg_targets = np.zeros((3,4))
        for i in range(4):
            leg_affix = self.getLegAffix(t, i)
            height = self.getLegHeight(leg_affix)
            wished_pos = self.robot_model.trunk_from_leg[i].dot(np.concatenate((self.restingPose,[1])))
            wished_pos[2] += height
            leg_targets[:,i] = wished_pos[:3]
        return leg_targets

class CartesianWalkEngine(ParametricWalkEngine):
    def getLegTargets(self, t):
        leg_targets = np.zeros((3,4))
        for i in range(4):
            leg_affix = self.getLegAffix(t, i)
            height = self.getLegHeight(leg_affix)
            nextX = self.getLegX(leg_affix)
            nextY = self.getLegY(leg_affix)
            wished_pos = self.robot_model.trunk_from_leg[i].dot(np.concatenate((self.restingPose,[1])))
            wished_pos[0] += nextX
            wished_pos[1] += nextY
            wished_pos[2] += height
            leg_targets[:,i] = wished_pos[:3]
        return leg_targets

class RotationWalkEngine(ParametricWalkEngine):
    def getLegTargets(self, t):
        leg_targets = np.zeros((3,4))
        for i in range(4):
            leg_affix = self.getLegAffix(t, i)
            height = self.getLegHeight(leg_affix)
            nextDir = self.getLegDir(leg_affix)
            wished_pos = self.robot_model.trunk_from_leg[i].dot(np.concatenate((self.restingPose,[1])))
            if i == 0:
                wished_pos[0] += nextDir
            if i == 1:
                wished_pos[0] += nextDir
            if i == 2:
                wished_pos[0] -= nextDir
            if i == 3:
                wished_pos[0] -= nextDir
            wished_pos[2] += height
            leg_targets[:,i] = wished_pos[:3]
        return leg_targets

class OmniDirectionalWalkEngine(ParametricWalkEngine):
    def getLegTargets(self, t):
        leg_targets = np.zeros((3,4))
        for i in range(4):
            leg_affix = self.getLegAffix(t, i)
            height = self.getLegHeight(leg_affix)
            nextX = self.getLegX(leg_affix)
            nextY = self.getLegY(leg_affix)
            nextDir = self.getLegDir(leg_affix)
            wished_pos = self.robot_model.trunk_from_leg[i].dot(np.concatenate((self.restingPose,[1])))
            if i == 0:
                wished_pos[0] += nextDir
            if i == 1:
                wished_pos[0] += nextDir
            if i == 2:
                wished_pos[0] -= nextDir
            if i == 3:
                wished_pos[0] -= nextDir
            wished_pos[0] += nextX
            wished_pos[1] += nextY
            wished_pos[2] += height
            leg_targets[:,i] = wished_pos[:3]
        return leg_targets


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
    def __init__(self, knots):
        """
        Parameters
        ----------
        knots : list((float,float))
            The list of couples (time, position)
        """
        self.knots = knots

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
            vel = (nextPos - srcPos) / (nextT -srcT)
            pos = srcPos + vel * (t-srcT)
            break
        return pos, vel

    def getDuration(self):
        return self.knots[-1][0]

class LinearSplinePos(CompositeTrajectory):
    def __init__(self, path):
        splines = {}
        with open(path) as splineFile:
            splines = json.load(splineFile)
        self.trajectories = []
        for key in ["x", "y", "z"]:
            self.trajectories.append(LinearSpline(splines[key]))
