import argparse
import json
import math
import numpy as np
import pybullet as p
import sys
from time import sleep, time
import control as ctrl

class Simulation:
    def __init__(self, dt = 0.01):
        # PyBullet initialization
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("resources/plane.urdf")

        # Loading robot
        robotInitialPos = [0.0, 0, 0.025]
        robotOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.holo = p.loadURDF("resources/holo/robot.urdf", robotInitialPos, robotOrientation)

        # Id of the wheels joints
        self.wheelIds = [
            p.getJointInfo(self.holo, 1)[0],
            p.getJointInfo(self.holo, 22)[0],
            p.getJointInfo(self.holo, 43)[0]
        ]

        # Id of the wheels
        self.wheels = np.array([0., 0., 0.])

        # Time management
        p.setRealTimeSimulation(0)
        self.t = 0
        self.dt = dt
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        self.last_tick = None

    def updateStatus(self):
        pos = p.getLinkState(self.holo, 0)[0]
        orientation = p.getEulerFromQuaternion(p.getLinkState(self.holo, 0)[1])
        robotYaw = orientation[2]
        self.robot = np.array([pos[0], pos[1], robotYaw])

    def setWheelSpeed(self,wheel_control):
        self.wheel_control = wheel_control;

    def tick(self):
        self.wheels += self.dt*np.asarray(self.wheel_control).flatten()

        for w in range(3):
            p.setJointMotorControl2(
                self.holo, self.wheelIds[w], p.POSITION_CONTROL, self.wheels[w])

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
    trajectory = parser.add_mutually_exclusive_group(required=True)
    trajectory.add_argument("--fixed", type=float, nargs=3,
                            metavar=("dstX", "dstY", "vTheta"),
                            help="Fixed target")
    trajectory.add_argument("--linear", type=float, nargs=3,
                            metavar=("vx", "vy", "vTheta"),
                            help="Fixed speed trajectory")
    trajectory.add_argument("--bangbang", type=float, nargs=7,
                            metavar=("srcX", "srcY", "srcTheta",
                                     "dstX", "dstY", "vTheta","startT"),
                            help="Uses bangbang trajectory on three dimensions")
    trajectory.add_argument("--bangbangWheel", type=float, nargs=7,
                            metavar=("srcX", "srcY", "srcTheta",
                                     "dstX", "dstY", "vTheta","startT"),
                            help="Uses bangbang trajectory based on wheels speed")
    trajectory.add_argument("--holoSquare", type=float, nargs=1,
                            metavar="size", help="Draws a square using holonomic motion")
    trajectory.add_argument("--directSquare", type=float, nargs=1,
                            metavar="size", help="Draws a square using direct motion")
    trajectory.add_argument("--linearSpline", type=str, nargs=1,
                            metavar="file.json", help="Follows a linear spline")
    trajectory.add_argument("--cubicSpline", type=str, nargs=1,
                            metavar="file.json",
                            help="Follows a cubic spline with derivative provided")
    control = parser.add_mutually_exclusive_group()
    control.add_argument("--openLoop", action="store_true",default =False)
    control.add_argument("--closeLoop", action="store_true",default =False)
    control.add_argument("--feedForward", action="store_true",default =False)
    parser.add_argument("-c", "--configFile", type=str, default="default_parameters.json")
    parser.add_argument("--duration",type=float, default = -1,
                        help= "Duration of the simulation [s]")
    args = parser.parse_args()
    # Loading parameters
    params = {}
    with open(args.configFile) as configFile:
        params = json.load(configFile)
    maxSpeed = params["maxSpeed"]
    maxAcc = params["maxAcc"]
    maxWheelSpeed = params["maxWheelSpeed"]
    maxWheelAcc = params["maxWheelAcc"]
    maxThetaSpeed = params["maxThetaSpeed"]
    maxThetaAcc = params["maxThetaAcc"]
    kp = params["kp"]
    ki = params["ki"]
    kd = params["kd"]
    # Loading trajectory
    trajectory = None
    if args.fixed:
        trajectory = ctrl.FixedTarget(np.array(args.fixed))
    if args.linear:
        trajectory = ctrl.LinearTarget(np.array(args.linear))
    if args.bangbang:
        bb_args = args.bangbang
        trajectory = ctrl.BangBangPose2D(np.array(bb_args[:3]),
                                         np.array(bb_args[3:6]), bb_args[6],
                                         maxSpeed, maxAcc,
                                         maxThetaSpeed, maxThetaAcc)
    if args.bangbangWheel:
        bb_args = args.bangbangWheel
        trajectory = ctrl.BangBangWheel(np.array(bb_args[:3]),
                                        np.array(bb_args[3:6]), bb_args[6],
                                        maxWheelSpeed, maxWheelAcc)
    if args.holoSquare:
        trajectory = ctrl.HoloSquare(args.holoSquare[0],
                                       maxSpeed, maxAcc,
                                       maxThetaSpeed, maxThetaAcc)
    if args.directSquare:
        trajectory = ctrl.DirectSquare(args.directSquare[0],
                                       maxSpeed, maxAcc,
                                       maxThetaSpeed, maxThetaAcc)
    if args.linearSpline:
        trajectory = ctrl.LinearSplinePose2D(args.linearSpline[0])
    if args.cubicSpline:
        trajectory = ctrl.CubicSplinePose2D(args.cubicSpline[0])
    controller = None
    if not args.openLoop:
        controller = ctrl.PIDController(kp, ki, kd, maxSpeed, maxThetaSpeed)
    # Launching simulation
    simulation = Simulation()
    simulation.updateStatus()
    out = open("data.csv", "w")
    last_target = np.array([0,0,0])
    print("t,target_x,target_y,target_theta,pos_x,pos_y,pos_theta,pos_w0,pos_w1,pos_w2,target_vx,target_vy,target_vtheta", file=out)
    while args.duration < 0 or simulation.t < args.duration:
        target = trajectory.getTargetPos(simulation.t)
        cartVel = trajectory.getTargetVel(simulation.t)
        robot = simulation.robot
        if args.openLoop:
            # In openLoop, there is no access to the position of the robot
            robot = last_target
        elif args.closeLoop:
            cartVel = controller.step(simulation.t, simulation.robot, target)
        else:#FeedForward
            cartVel = controller.step(simulation.t, simulation.robot, target, cartVel)
        wheelSpeeds = ctrl.cartVelToWheelVel(robot, cartVel)
        simulation.setWheelSpeed(wheelSpeeds)
        simulation.tick()
        simulation.updateStatus()
        last_target = target
        print(cartVel)
        print(("{:3f},"*12 +"{:3f}").format(
            simulation.t,target[0],target[1],target[2],
            simulation.robot[0], simulation.robot[1],simulation.robot[2],
            wheelSpeeds.item(0), wheelSpeeds.item(1),wheelSpeeds.item(2),
            cartVel.item(0), cartVel.item(1), cartVel.item(2)), file=out)

    out.close()
