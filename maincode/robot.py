#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive,interfaces
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel

class MyRobot(wpilib.TimedRobot):
    """Main robot class."""


    def robotInit(self):
        self.joystick = Joystick(0)
        self.controller = interfaces.GenericHID(1)
        top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.arm = CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless)


        top_right.setInverted(True)
        bottom_right.setInverted(True)
        top_left.setInverted(True)
        bottom_right.setInverted(False)

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
        self.comp = Compressor(0, PneumaticsModuleType.CTREPCM)
        self.comp.enableDigital()

        self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)


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

        if self.controller.getRawButton(5):
            self.doubles.set(DoubleSolenoid.Value.kForward)
        elif self.controller.getRawButton(6):
            self.doubles.set(DoubleSolenoid.Value.kReverse)
        else:
            self.doubles.set(DoubleSolenoid.Value.kOff)

        # self.driver()
        joy_y = self.joystick.getY()
        joy_x = self.joystick.getX()
        joy_z = self.joystick.getZ()

        if math.fabs(self.joystick.getY())<0.3:
            joy_y = 0
        if math.fabs(self.joystick.getX())<0.3:
            joy_x = 0
        if math.fabs(self.joystick.getZ())<0.3:
            joy_z = 0
        self.mec_drive.driveCartesian(-joy_y * 0.4, joy_x * 0.4, joy_z * 0.3)


        if self.controller.getRawButton(3):
            self.arm.set(0.8)
        elif self.controller.getRawButton(4):
            self.arm.set(-0.8)


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
