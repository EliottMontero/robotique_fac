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

class Simulation:
    def __init__(self, joint_space, target, dt = 0.01):
        """
        Parameters
        ----------
        joint_space : bool
            Is the target in joint_space
        target : np.array (4) or None
            Initial target, if None, target cannot be controled through user parameters
        """
        # PyBullet initialization
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # Loading ground
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        # Loading target
        self.tool_target_id = p.loadURDF("resources/tool_target.urdf", [0,0,0], useFixedBase=True)
        self.tool_pos_id = p.loadURDF("resources/tool_position.urdf", [0,0,0], useFixedBase=True)
        # Loading robot
        self.robot = p.loadURDF("resources/leg.urdf",useFixedBase=True)
        self.nb_joints = p.getNumJoints(self.robot)
        self.target = np.array([0.0]* self.nb_joints)
        self.joint_target = np.array([0.0]* self.nb_joints)

        # Time management
        p.setRealTimeSimulation(0)
        self.t = 0
        self.dt = dt
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        self.last_tick = None
        self.manual_target = False
        if target is not None:
            self.manual_target = True
            self.addUserDebugParameters(joint_space, target)

    def addUserDebugParameters(self, joint_space, target):
        self.debug_targets = []
        if joint_space:
            for i in range(4):
                param_id = p.addUserDebugParameter("Target q{:}".format(i),
                                                   -np.pi, np.pi, target[i])
                self.debug_targets.append(param_id)
        else:
            names = ['X', 'Y', 'Z', 'Dir']
            for i in range(4):
                val_min = -2
                val_max = 2
                if i == 2:
                    val_min = 0
                if i == 3:
                    val_min = -1
                    val_max = 1
                param_id = p.addUserDebugParameter("Target {:}".format(names[i]),
                                                   val_min, val_max, target[i])
                self.debug_targets.append(param_id)

    def readUserDebugParameters(self):
        for i in range(len(self.debug_targets)):
            self.target[i] = p.readUserDebugParameter(self.debug_targets[i])

    def updateStatus(self):
        self.joints = np.array([0.0] * self.nb_joints)
        for i in range(self.nb_joints):
            self.joints[i] = p.getJointState(self.robot, i)[0]
        if self.manual_target:
            self.readUserDebugParameters()
        # print("Current Q: {:}".format(self.joint_target))

    def setWheelSpeed(self,wheel_control):
        self.wheel_control = wheel_control;

    def setJointTarget(self, target):
        self.joint_target = target

    def tick(self):
        # print("Target Q: {:}".format(self.joint_target))
        for i in range(self.nb_joints):
            ctrl_mode = p.POSITION_CONTROL
            p.setJointMotorControl2(self.robot, i, ctrl_mode, self.joint_target[i])

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


    def setTargetPos(self, pos):
        p.resetBasePositionAndOrientation(self.tool_target_id, pos, [0,0,0,1])

    def setToolPos(self, pos):
        p.resetBasePositionAndOrientation(self.tool_pos_id, pos, [0,0,0,1])

    def drawPoint(self, pos, color):
        for dim in range(3):
            offset = [0.0,0,0]
            offset[dim] = 0.02
            start = pos - offset
            end = pos + offset
            p.addUserDebugLine(start, end, color, 3, self.dt)

if __name__ == "__main__":
    # Reading arguments
    parser = argparse.ArgumentParser()
    commandType = parser.add_mutually_exclusive_group(required=True)
    commandType.add_argument("--joint-space", action="store_true",
                             help="Target trajectory is provided in joint space directly")
    commandType.add_argument("--analytical-mgi", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "analyticalMGI provides the joint target")
    commandType.add_argument("--jacobian-inverse", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "jacobian inverse method is used to provide the joint target")
    commandType.add_argument("--jacobian-transposed", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "jacobian transposed method is used to provide the joint target")

    parser.add_argument("--target",type=float, nargs=4, required = False,
                        help= "Target for the robot (either joint-space or operational-space)")
    parser.add_argument("--duration",type=float, default = -1,
                        help= "Duration of the simulation [s]")
    parser.add_argument("--trajectory-type",type=str, default = "manual",
                        help= "The type of trajectory to use")
    parser.add_argument("--trajectory-path",type=str, default = "trajectory.json",
                        help="The path to the trajectory")
    parser.add_argument("--cyclic", action="store_true",
                        help= "Restart chrono once duration of the trajectory has been reached")
    args = parser.parse_args()

    target = np.array([0,0,0,0])
    if args.target:
        target = np.array(args.target)
    elif not args.joint_space:
        target = np.array([0,1.0,1.0,0.0])

    trajectory = None
    if args.trajectory_type != "manual":
        if args.trajectory_type == "linear_spline":
            trajectory  = ctrl.LinearSplinePose(args.trajectory_path)
        if args.trajectory_type == "cubic_spline":
            trajectory  = ctrl.CubicSplinePose(args.trajectory_path)
        target = None

    # Launching simulation
    simulation = Simulation(args.joint_space, target)
    simulation.updateStatus()
    robot_model = ctrl.LegModel()
    # Writing data
    out = open("data.csv", "w")
    print("t",file=out,end="")
    for variable in ["q","o"]:
        for data_type in ["target", "measured"]:
            # When in joint space mode, there are no target in operational space
            if args.joint_space and variable == "o" and data_type == "target":
                continue
            for i in range(4):
                print(",{:}_{:}{:}".format(data_type,variable,i),file=out,end="")
    print("\n",file=out,end="")
    last_target = np.array([0,0,0])
    # Main loop
    while args.duration < 0 or simulation.t < args.duration:
        target_speed = np.array([0,0,0,0])
        if trajectory is not None:
            traj_time = simulation.t
            if (args.cyclic):
                traj_time = traj_time % trajectory.getDuration()
            simulation.target, target_speed = trajectory.getTarget(traj_time)
        joint_target = None
        if args.joint_space:
            joint_target = simulation.target
        if args.analytical_mgi:
            joint_target, nb_sols = robot_model.computeAnalyticalMGI(simulation.target)
        if args.jacobian_inverse:
            joint_target = ctrl.searchJacInv(robot_model, simulation.joints, simulation.target)
        if args.jacobian_transposed:
            joint_target = ctrl.searchJacTransposed(robot_model, simulation.joints, simulation.target)
        if not joint_target is None:
            simulation.setJointTarget(joint_target)
        simulation.tick()
        simulation.updateStatus()

        tool_pos = robot_model.computeMGD(simulation.joints)

        if not args.joint_space:
            simulation.setTargetPos(simulation.target[:3])
        if tool_pos is not None:
            simulation.setToolPos(tool_pos[:3])
        print("{:3f}".format(simulation.t), file=out, end="")
        dump_vectors = [simulation.joint_target, simulation.joints]
        if not args.joint_space:
            dump_vectors.append(simulation.target)
        dump_vectors.append(tool_pos)
        for v in dump_vectors:
            print(v)
            for i in range(4):
                if v is None:
                    print(",NA", file=out, end="")
                else:
                    print(",{:3f}".format(v[i]), file=out, end="")
        print("\n",file=out,end="")
