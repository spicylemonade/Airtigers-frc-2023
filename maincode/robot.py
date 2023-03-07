#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel

import numpy as np
scored, mobility, balanced = False, False, False;


# class Robot:
#     # Wheels
#     wheel_radius = 3 * (0.0254);  # Radius of the wheel
#     wheel_mass = 0.5 * (1 / 2.20462);  # mass of the wheel
#     I_wheel = 1 / 2 * wheel_mass * wheel_radius ** 2  # Moment of Inertia of the robot wheels
#     wheel_friction = 0.1;  # Damping coefficient of the robot wheels
#
#     # Robot Chasis
#     base_length = 27 * (0.0254);  # Length of the robot (INCORRECT NEED TO COMPUTE)
#     base_width = 24 * (0.0254);  # width of the robot  (INCORRECT NEED TO COMPUTE)
#     base_height = 4 * (0.0254);  # height of the chassis  (INCORRECT NEED TO COMPUTE)
#     robot_mass = 125 * (1 / 2.20462);  # Mass of the robot in Kg
#     robot_height = 36 * (0.0254);  # height of the robot (wheels not included)  (INCORRECT NEED TO COMPUTE)
#
#     # Arm
#     arm_length_min = 25 * (0.0254);  # minimum arm length
#     arm_length_max = 49 * (0.0254);  # maximum arm length
#     Total_arm_mass = 12 * (0.453592);  # mass of the arm   (INCORRECT NEED TO COMPUTE)
#     Extended_arm_mass = 4 * (0.453592);  # mass of the arm   (INCORRECT NEED TO COMPUTE)
#     arm_friction = 0.5;  # rotational friction in the arm
#     arm_width = 3 * (0.0254);  # width of the arm
#     rotate_arm_width = 2 * (0.0254);  # width of the rotating arm
#     slide_friction = 5;
#     I_arm = 0.3;  # Inertia of the arm (INCORRECT NEED TO COMPUTE)
#     pistonForce = (60 * math.pi * (1.5 / 2) ** 2) * 4.4482189159;  # Force of the piston
#     maxArmTorque = 166.4;
#     # Roller Claw
#     claw_mass = 10 * (0.453592);  # Mass of the roller claw
#     claw_length = 12 * (0.0254);  # Length of the claw
#     I_claw = 0.01;  # Moment of inertia of the claw (INCORRECT NEED TO COMPUTE)
#
#
# def scoring_Controller(x, xDot, arm_theta, arm_thetaDot, l1, l1Dot):
#     global scored;
#     # To score we must drive forward, position the arm, and release the cube on the top level
#     x_ref = 0;  # this is the target position to score (DRIVE ROBOT TO THIS POINT)
#     arm_theta_ref = 0;  # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
#     arm_length_ref = Robot.arm_length_min;  # this is the target arm length
#
#     K_drive = -0.3;  # this is the control gain for driving the robot during scroring
#     K_speed = 0.4;  # this is the control gain for the robots speed.
#     K_arm_orient = 1.5;  # this is the control gain for the robot arm orientation
#     K_arm_ang_vel = 0.5;  # this is the control gain for the robot's arm angulat velocity
#     U = np.zeros((6, 1));  # Predefine the robot's control input as a vector
#
#     # compute input
#     ref = np.array([x_ref, 0, arm_theta_ref, 0, arm_length_ref, 0]);  # vector of references
#     X = np.array([x, xDot, arm_theta, arm_thetaDot, l1, l1Dot]);  # vector of actual states
#     err = ref - X;
#     # need to make sure to set max arm length after arm is at certain angle
#     # if arm theta ref within a certain range?
#
#     U[0] = K_drive * err[0] + K_speed * err[1];
#     U[1] = -U[0];
#     U[2] = K_arm_orient * err[2] + K_arm_ang_vel * err[3];
#
#     if (arm_theta <= 5 * (math.pi) / 180):
#         ref[4] = Robot.arm_length_max
#         # pdb.set_trace()
#     if (ref[4] == Robot.arm_length_max):
#         U[3] = Robot.pistonForce;
#     elif (ref[4] == Robot.arm_length_min):
#         U[3] = -Robot.pistonForce;
#
#     # Check if robot has scored (ARE THE STATES EQUAL TO THE REFERENCE VALUES)
#     if (np.abs(err[0]) <= 0.065 and np.abs(err[1]) <= 0.15):
#         scored = True;
#     # 		pdb.set_trace();
#     # 	pdb.set_trace();
#     return U
#
#
# def mobility_Controller(x, xDot, arm_theta, arm_thetaDot, l1, l1Dot):
#     # To get mobility points we must drive backwards over the charging station and leave the community
#     x_ref = (96.75 + 24) * (0.0254);  # this is the target position to score (DRIVE ROBOT TO THIS POINT)
#     arm_theta_ref = 70 * (math.pi / 180);  # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
#     arm_length_ref = Robot.arm_length_min;  # this is the target arm length
#
#     K_drive = -1.5;  # this is the control gain for driving the robot during scroring
#     K_speed = 0.6;  # this is the control gain for the robots speed.
#     K_arm_orient = 1;  # this is the control gain for the robot arm orientation
#     K_arm_ang_vel = 0.5;  # this is the control gain for the robot's arm angulat velocity
#
#     U = np.zeros((6, 1));  # Predefine the robot's control input as a vector
#     # compute input
#     ref = np.array([x_ref, 0, arm_theta_ref, 0, arm_length_ref, 0]);  # vector of references
#     X = np.array([x, xDot, arm_theta, arm_thetaDot, l1, l1Dot]);  # vector of actual states
#     err = ref - X;
#
#     U[0] = K_drive * err[0] + K_speed * err[1];
#     U[1] = -U[0];
#     U[2] = K_arm_orient * err[2] + K_arm_ang_vel * err[3];
#
#     # 	pdb.set_trace()
#     if (ref[4] == Robot.arm_length_max):
#         U[3] = Robot.pistonForce;
#     elif (ref[4] == Robot.arm_length_min):
#         U[3] = -Robot.pistonForce;
#
#     if (np.abs(err[0]) <= 0.098 * x_ref and np.abs(err[1]) <= 0.005):
#         mobility = True;
#     # sim2.bal(98,56,110)
#
#     return U
#
#
# def Inputs(x, xDot, arm_theta, arm_thetaDot, l1, l1Dot):
#     # x is robots position (encoder)
#     # xDot velocity
#     # arm_theta is arm angle
#     # arm_thetaDot is angular velocity
#     # l1 is arm length(1 of 2 when button is pressed)
#     # l1dot is how fast arm length is changing
#     global scored;
#     if (not scored):
#         # the current priority is to score, so implement a controller for scoring
#         U = scoring_Controller(x, xDot, arm_theta, arm_thetaDot, l1, l1Dot);
#     elif (not mobility):
#         # the current priority is to get mobility point, so implement a controller to leave the grid
#         U = mobility_Controller(x, xDot, arm_theta, arm_thetaDot, l1, l1Dot);
#         # 		pdb.set_trace();
#         # 		pdb.set_trace();
#         # 	pdb.set_trace();
#     if (U[2] >= Robot.maxArmTorque):
#         U[2] = Robot.maxArmTorque;
#     return U;
#
#
# def bal(Kdx, Kpt, Kdt):
#     # Define Robot Parameters
#     mass = 125 * (1 / 2.20462);  # Mass of the robot in Kg
#     l_robot = 3 * (12) * (2.54 / 100);  # Length of the robot in meters
#     w_robot = 2 * (12) * (2.54 / 100);  # Width of the robot in meters
#
#     # Define Charge Station Parameters
#     h_charge = 12.5 * (2.54 / 100);  # Height of center of the charge station in meters
#     w_charge = 8 * (12) * (2.54 / 100);  # Width of the charge station in meters
#     l_charge = 4 * (12) * (2.54 / 100);  # Length of the charge station in meters
#
#     # Define Initial conditions
#     x_IC = l_charge;  # Initial robot position on the charge station in meters
#     xDot_IC = 0;  # Initial robot velocity in m/s
#     theta_IC = -15 * (2 * math.pi / 360);  # Initial orientation of the charging station
#     thetaDot_IC = 0;  # Initial angular velocity of the charging station
#     IC = np.array([x_IC, xDot_IC, theta_IC, thetaDot_IC]);
#
#     # Define the simulation duration
#     startTime = 0;  # This is the starting time of the simulation (seconds)
#     endTime = 20;  # This is the end time of the simulation
#     dt = 0.005;  # This is the time step difference
#     time = np.arange(startTime, endTime + dt, dt)
#     #   pdb.set_trace()
#
#     # Simulate the dynamics
#     x, xDot, theta, thetaDot, u = balance(IC, time, mass, dt, l_robot, w_robot, l_charge, w_charge, h_charge, Kdx, Kpt,
#                                           Kdt)
#
#
# def balance(IC, time, m, dt, l_robot, w_robot, l_charge, w_charge, h_charge, KDx, KPt, KDt):
#     g = -9.81;  # gravity
#     Cd = 50;
#     # Pre-define arrays
#     x = np.zeros(len(time));
#     xDot = np.zeros(len(time));
#     theta = np.zeros(len(time));
#     thetaDot = np.zeros(len(time));
#     u = np.zeros(len(time));
#
#     # Assign Initial Conditions
#     x[0] = IC[0];
#     xDot[0] = IC[1];
#     theta[0] = IC[2];
#     thetaDot[0] = IC[3];
#
#     # Controller
#     KPx = 0;
#     #   KDx = -60;
#     #   KPt = 15;
#     #   KDt = 95;
#     # pdb.set_trace()
#     """ Simulate using Euler's Method """
#     for n in range(len(time) - 1):
#         leftRamp = (h_charge - l_charge * np.sin(theta[n]));
#         rightRamp = (h_charge + l_charge * np.sin(theta[n]));
#         # Normal Force
#         N = m * (xDot[n] * thetaDot[n] + g * np.cos(theta[n]) * (1 - x[n] ** 2 / Inertia(x[n], m, l_robot, w_robot)));
#         # Drive Force
#         u[n] = 1 * (-m * g * np.sin(theta[n]) - (m) * x[n] * thetaDot[n] ** 2) + KDx * xDot[n] + KPt * np.sin(
#             theta[n]) + KDt * thetaDot[n];
#         # KDx: how aggressive is my velocity to balance
#         # KPt: how to respond to ramp angle
#         # KDt: how to respond to ramp angular velocity
#
#         if ((leftRamp <= 0 and thetaDot[n] > 0) or (rightRamp <= 0) and thetaDot[n] < 0):
#             theta[n + 1] = theta[n];  # pdb.set_trace();
#         else:
#             theta[n + 1] = theta[n] + dt * (thetaDot[n])
#
#         if ((x[n] <= (-l_charge * np.cos(theta[n])) and xDot[n] < 0) or (
#                 x[n] >= (l_charge * np.cos(theta[n])) and xDot[n] > 0)):
#             x[n + 1] = x[n]
#         else:
#             x[n + 1] = x[n] + 1 * dt * (xDot[n])
#
#         # x[n+1] = x[n]+dt*xDot[n];
#         xDot[n + 1] = xDot[n] + 1 * dt * (1 / m * (u[n] + m * g * np.sin(theta[n])) + x[n] * thetaDot[n] ** 2);
#         # theta[n+1] = theta[n] + dt*thetaDot[n];
#         thetaDot[n + 1] = thetaDot[n] + dt * (
#                 1 / Inertia(x[n], m, l_robot, w_robot) * (m * g * x[n] * np.cos(theta[n])) - Cd * thetaDot[n]);
#         insec = 0.005 * n
#         timesec = 0.005 * len(time)
#         if ((insec + 10 <= timesec) and insec <= 20):
#             if ((x[n + int((10) / 0.005)] - theta[n] <= 0.1) and (x[n + int((10) / 0.005)] - theta[n] >= -0.1)):
#                 print("dx: ", KDx, "pt: ", KPt, "dt: ", KDt)
#
#         # pdb.set_trace();
#
#     return x, xDot, theta, thetaDot, u
#

class MyRobot(wpilib.TimedRobot):
    """Main robot class."""
    joystick: Joystick = None
    mec_drive: drive.MecanumDrive = None

    def driver(self):
        joy_y = self.joystick.getY()
        joy_x = self.joystick.getX()
        joy_z = self.joystick.getZ()
        if math.fabs(self.joystick.getY()):
            joy_y = 0
        if math.fabs(self.joystick.getX()):
            joy_x = 0
        if math.fabs(self.joystick.getZ()):
            joy_z = 0
        self.mec_drive.driveCartesian(-joy_y * 0.4, joy_x * 0.4, joy_z * 0.3)

    def robotInit(self):
        top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)

        top_right.setInverted(True)
        bottom_right.setInverted(True)

        self.mec_drive = drive.MecanumDrive(top_left, bottom_left, top_right, bottom_right)

        tlm = top_left.getEncoder()
        blm = bottom_left.getEncoder()
        trm = top_right.getEncoder()
        brm = bottom_right.getEncoder()

        tlm.setPosition(0.0)
        blm.setPosition(0.0)
        trm.setPosition(0.0)
        brm.setPosition(0.0)
        """Robot-wide initialization code should go here."""
        # self.comp = Compressor(0, PneumaticsModuleType.CTREPCM)
        # self.comp.enableDigital()
        #
        # self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        # U = Inputs(0, 0, 0, 0, 0, 0)
        # u[0] and u[1] for drive
        # u[2] for arm
        # u[3] piston
        pass

    def disabledInit(self):
        """Called only at the beginning of disabled mode."""
        # self.logger.info("%d loops / %f seconds", self.loops, self.timer.get())

    def disabledPeriodic(self):
        """Called every 20ms in disabled mode."""
        pass

    def teleopInit(self):

        """Called only at the beginning of teleoperated mode."""
        # self.loops = 0
        # self.timer.reset()
        # self.timer.start()

    def teleopPeriodic(self):
        """Called every 20ms in teleoperated mode"""

        # if self.j1.getRawButton(2):
        #     self.doubles.set(DoubleSolenoid.Value.kForward)
        # elif self.j1.getRawButton(3):
        #     self.doubles.set(DoubleSolenoid.Value.kReverse)
        # else:
        #     self.doubles.set(DoubleSolenoid.Value.kOff)

        self.driver()
        # Move a motor with a Joystick

        # self.motor.set(self.lstick.getY())
        #
        # # Print out the number of loop iterations passed every second
        # self.loops += 1
        # if self.timer.advanceIfElapsed(1):
        #     self.logger.info("%d loops / second", self.loops)
        #     self.loops = 0


if __name__ == "__main__":
    wpilib.run(MyRobot)
