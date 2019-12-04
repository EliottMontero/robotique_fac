#!/usr/bin/env python3

import argparse
import json
import math
import numpy as np
import pybullet as p
import pybullet_data
import sys
from time import sleep, time
import control as ctrl
from homogeneous_transform import invertTransform

class Simulation:
    """
    Members
    -------
    mode: bool
        Control mode
    joints : dictionary
        maps joint names to the joint_id used by pyBullet
    joint_id_to_name: dictionary
        maps joint_id with the name it has been given
    joint_id_to_dof_idx : dictionary
        maps the id of the joint according to pybullet to the index of the dof
    dof_idx_to_joint_id : dictionary
        maps the index of the dof to the id of the joint according to pybullet
    """
    def __init__(self, mode, parameters, dt = 0.01):
        self.mode = mode
        self.parameters = parameters
        # PyBullet initialization
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # Loading ground
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        # Loading robot
        robotStartPos = [0, 0, 0.5]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0,-3*np.pi/4])
        self.robot = p.loadURDF("quadruped4/robot.urdf",
                                robotStartPos, robotStartOrientation)

        # Retrieving joints and frames
        self.joints = {}
        self.joint_id_to_name = {}
        self.frames = {}
        self.joint_id_to_dof_idx = {}
        self.dof_idx_to_joint_id = {}


        # Collecting the available joints
        for k in range(p.getNumJoints(self.robot)):
            jointInfo = p.getJointInfo(self.robot, k)
            name = jointInfo[1].decode('utf-8')
            if '_fixing' not in name:
                if '_frame' in name:
                    self.frames[name] = k
                else:
                    joint_id = k
                    dof_idx = len(self.joints)
                    self.joint_id_to_dof_idx[joint_id] = dof_idx
                    self.dof_idx_to_joint_id[dof_idx] = joint_id
                    self.joints[name] = joint_id
                    self.joint_id_to_name[joint_id] = name
                    print("Adding joint '{:}' with dof_idx {:}".format(name, dof_idx))

        print('* Found '+str(len(self.joints))+' DOFs')
        print('* Found '+str(len(self.frames))+' frames')

        self.joint_pos = np.zeros(len(self.joints))
        self.joint_target = np.zeros(len(self.joints))

        self.addUserDebugParameters()

        # Time management
        p.setRealTimeSimulation(0)
        self.t = 0
        self.dt = dt
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        self.last_tick = None

    def addDebugParam(self, name, interval, initial_value):
        self.debug_parameters[name] = p.addUserDebugParameter(name, interval[0], interval[1], initial_value)

    def addUserDebugParameters(self):
        self.debug_targets = []
        self.debug_parameters = {}
        if self.mode == "None":
            for name, joint_id in self.joints.items():
                dof_idx = self.joint_id_to_dof_idx[joint_id]
                print ("Adding user debug param for: {:} with joint_id {:} and dof idx {:}".format(name,joint_id, dof_idx))
                param_id = p.addUserDebugParameter("Target {:}".format(name),
                                                   -np.pi, np.pi,
                                                   self.joint_target[dof_idx])
                self.debug_targets.append(param_id)
        else:
            self.addDebugParam("speed", [10**-3, 16.0], self.parameters["speed"])
            self.addDebugParam("restX", [-0.1, 0.1], self.parameters["restX"])
            self.addDebugParam("restY", [0.0, 0.2], self.parameters["restY"])
            self.addDebugParam("restZ", [-0.2, 0.0], self.parameters["restZ"])
            if self.mode not in ["Fixed","Trajectory"]:
                self.addDebugParam("flyingRatio", [0.0, 0.25], self.parameters["flyingRatio"])
                self.addDebugParam("stepHeight", [0.0, 0.1], self.parameters["stepHeight"])
            if self.mode in ["Cartesian","OmniDirectional"]:
                self.addDebugParam("stepX", [-0.1, 0.1], self.parameters["stepX"])
                self.addDebugParam("stepY", [-0.1, 0.1], self.parameters["stepY"])
            if self.mode in ["Rotation","OmniDirectional"]:
                self.addDebugParam("stepDir", [-np.pi/2, np.pi/2], self.parameters["stepDir"])

    def readUserDebugParameters(self):
        if self.mode == "None":
            for dof_idx in range(len(self.debug_targets)):
                self.joint_target[dof_idx] = p.readUserDebugParameter(self.debug_targets[dof_idx])
        else:
            for key, param_id in self.debug_parameters.items():
                self.parameters[key] = p.readUserDebugParameter(param_id)

    def updateStatus(self):
        for i in range(len(self.joints)):
            self.joint_pos[i] = p.getJointState(self.robot, i)[0]
        self.readUserDebugParameters()

    def tick(self):
        for dof_idx in range(len(self.joints)):
            ctrl_mode = p.POSITION_CONTROL
            joint_id = self.dof_idx_to_joint_id[dof_idx]
            joint_target = self.joint_target[dof_idx]
            joint_name = self.joint_id_to_name[joint_id]
            if joint_name[-1] != 'a' or joint_name == "leg2_a":
                joint_target = -joint_target
            p.setJointMotorControl2(self.robot, joint_id, ctrl_mode, joint_target)

        # Make sure that time spent is not too high
        now = time()
        if not self.last_tick is None:
            tick_time = now - self.last_tick
            sleep_time = self.dt - tick_time
            if sleep_time > 0:
                sleep(sleep_time)
            else:
                print("Time budget exceeded: {:}".format(tick_time), file=sys.stderr)
        self.last_tick = time()
        self.t += self.dt
        p.stepSimulation()


if __name__ == "__main__":
    # Reading arguments
    parser = argparse.ArgumentParser()

    parser.add_argument("--duration",type=float, default = -1,
                        help= "Duration of the simulation [s]")
    parser.add_argument("--splinePath",type=str, default = "trajectory.json",
                        help="The path to the spline used for the leg")
    parser.add_argument("--walkEngine", type=str, default="None",
                        choices = ["None","Fixed","Trajectory","LegLifter",
                                   "Rotation","Cartesian","OmniDirectional"],
                        help="Which type of walkEngine is used")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="The speed at which the walk motion is being played")
    parser.add_argument("--restX", type=float,
                        default=ctrl.defaultParameters["restX"],
                        help="The base position of leg extremity along 'x'-axis [m]")
    parser.add_argument("--restY", type=float,
                        default=ctrl.defaultParameters["restY"],
                        help="The base position of leg extremity along 'y'-axis [m]")
    parser.add_argument("--restZ", type=float,
                        default=ctrl.defaultParameters["restZ"],
                        help="The base position of leg extremity along 'z'-axis [m]")
    parser.add_argument("--flyingRatio", type=float,
                        default=ctrl.defaultParameters["flyingRatio"],
                        help="Ratio of time spent in the air for each leg")
    parser.add_argument("--stepHeight", type=float,
                        default=ctrl.defaultParameters["stepHeight"],
                        help="Maximal distance between the ground and the flying foot [m]")
    parser.add_argument("--stepX", type=float, default=0.0,
                        help="Expected motion of the trunk for a full walk period along x-axis[m]")
    parser.add_argument("--stepY", type=float, default=0.0,
                        help="Expected motion of the trunk for a full walk period along y-axis[m]")
    parser.add_argument("--stepDir", type=float, default=0,
                        help="Expected rotation of the trunk for a full walk period [rad]")
    args = parser.parse_args()

    parameters = {
        "speed" : args.speed,
        "restX" : args.restX,
        "restY" : args.restY,
        "restZ" : args.restZ,
        "flyingRatio" : args.flyingRatio,
        "stepHeight" : args.stepHeight,
        "stepX" : args.stepX,
        "stepY" : args.stepY,
        "stepDir" : args.stepDir
    }

    # Launching simulation
    simulation = Simulation(args.walkEngine, parameters)
    simulation.updateStatus()
    robot_model = ctrl.QuadrupedModel()
    walkEngine = None
    if args.walkEngine == "Fixed":
        walkEngine = ctrl.FixedWalkEngine(robot_model)
    elif args.walkEngine == "Trajectory":
        trajectory  = ctrl.LinearSplinePos(args.splinePath)
        walkEngine = ctrl.TrajectoryWalkEngine(trajectory, robot_model)
    elif args.walkEngine == "LegLifter":
        walkEngine = ctrl.LegLifter(robot_model)
    elif args.walkEngine == "Cartesian":
        walkEngine = ctrl.CartesianWalkEngine(robot_model)
    elif args.walkEngine == "Rotation":
        walkEngine = ctrl.RotationWalkEngine(robot_model)
    elif args.walkEngine == "OmniDirectional":
        walkEngine = ctrl.OmniDirectionalWalkEngine(robot_model)
    if walkEngine is not None:
        walkEngine.setParameters(parameters)
    # Main loop
    while args.duration < 0 or simulation.t < args.duration:
        target_speed = np.array([0,0,0,0])
        if walkEngine is not None:
            leg_targets = walkEngine.getLegTargets(simulation.t)
            simulation.joint_target = robot_model.analyticalMGI(leg_targets, simulation.joint_pos)
        simulation.tick()
        simulation.updateStatus()
        if walkEngine is not None:
            walkEngine.setParameters(simulation.parameters)
