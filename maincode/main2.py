#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive, interfaces, DigitalInput
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel
from navx import AHRS
import numpy as np


class MyRobot(wpilib.TimedRobot):
    """Main robot class."""
    preSet_middlePeg = 25.97  # confirm later
    preSet_topPeg = 28.5  # confirm later

    isExtended = False
    scored, mobility, balanced = False, False, False
    arm1 = False
    arm2 = False
    arm3 = False
    wheel_circ = 2 * math.pi * 4
    rev = 6.78

    def robotInit(self):

        wpilib.CameraServer.launch('vision.py:main')
        self.navx = AHRS.create_spi()
        self.joystick = Joystick(0)
        self.controller = interfaces.GenericHID(0)
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.arm = CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless)

        self.armlimit_bottom = DigitalInput(1)
        self.armlimit_top = DigitalInput(0)

        self.top_right.setInverted(True)
        self.bottom_left.setInverted(False)
        self.top_left.setInverted(False)
        self.bottom_right.setInverted(True)

        self.mec_drive = drive.MecanumDrive(self.top_left, self.bottom_left, self.top_right, self.bottom_right)

        self.tlm = self.top_left.getEncoder()
        self.blm = self.bottom_left.getEncoder()
        self.trm = self.top_right.getEncoder()
        self.brm = self.bottom_right.getEncoder()
        self.arm_encoder = self.arm.getEncoder()

        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        """Robot-wide initialization code should go here."""
        self.comp = Compressor(0, PneumaticsModuleType.CTREPCM)
        self.comp.enableDigital()

        self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)

        self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.arm_encoder.setPosition(0)

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        self.navx.reset()
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        if (self.tlm.getPosition() < (38 / (self.wheel_circ)) * self.rev) and (self.scored == False):
            # move forward 38 inches
            self.scoring(38)
        elif (self.tlm.getPosition() > (-(195 - 38) / (self.wheel_circ)) * self.rev) and (self.mobility == False):
            # community is 195", since nodes are sean as 38 inchesn, subtract 38
            self.mobilize(-(195 - 38))
        elif (self.mobility == True) and (self.navx.getRate() < 1 and self.navx.getRate() > 1) and (
                self.balanced == False):  # iffy rate
            self.mec_drive.driveCartesian(0.5 * -0.5, 0, 0)
        else:
            self.balancing()
        pass

    def disabledInit(self):
        """Called only at the beginning of disabled mode."""
        # self.logger.info("%d loops / %f seconds", self.loops, self.timer.get())

    def disabledPeriodic(self):
        """Called every 20ms in disabled mode."""
        pass

    def teleopInit(self):
        self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        # self.arm_encoder.setPosition(0)

        """Called only at the beginning of teleoperated mode."""
        # self.loops = 0
        # self.timer.reset()
        # self.timer.start()
    def teleopPeriodic(self):
        """Called every 20ms in teleoperated mode"""

        if self.controller.getRawButton(5):
            self.isExtended = True
            self.doubles.set(DoubleSolenoid.Value.kForward)
        elif self.controller.getRawButton(6):
            self.isExtended = False
            self.doubles.set(DoubleSolenoid.Value.kReverse)
        else:
            self.doubles.set(DoubleSolenoid.Value.kOff)

        # self.driver()
        joy_y = self.joystick.getY()
        joy_x = self.joystick.getX()
        joy_z = self.joystick.getZ()

        if math.fabs(self.joystick.getY()) < 0.3:
            joy_y = 0
        if math.fabs(self.joystick.getX()) < 0.3:
            joy_x = 0
        if math.fabs(self.joystick.getZ()) < 0.3:
            joy_z = 0
        self.mec_drive.driveCartesian(joy_y * -0.5, joy_x * 0.5,
                                      joy_z * 0.5)  # cant be on at same time as controll_rev()

        # self.logger.info(self.arm_encoder.getPosition())
        # self.logger.info(self.tlm.getPosition())
        self.logger.info(self.arm_encoder.getPosition())
        # middle node(cone) 25.97
        # top node (28.5)

        self.arm_controller()

        # Move a motor with a Joystick
        # self.controll_rev(0.3,2*math.pi*4)
        if self.controller.getRawButton(8):
            self.arm1 = True
            self.arm2 = False
            self.arm3 = False
        if self.controller.getRawButton(10):
            self.arm2 = True
            self.arm1 = False
            self.arm3 = False
        if self.controller.getRawButton(12):
            self.arm3 = True
            self.arm1 = False
            self.arm2 = False

    # def controll_rev(self,power,x):
    #     self.bottom_left.set(power*(((x*self.rev)-self.blm.getPosition())/(x*self.rev)))
    #     self.top_left.set(power*(((x*self.rev)-self.tlm.getPosition())/(x*self.rev)))
    #     self.bottom_right.set(power*(((x*self.rev)-self.brm.getPosition())/(x*self.rev)))
    #     self.top_right.set(power*(((x*self.rev)-self.trm.getPosition())/(x*self.rev)))
    def scoring(self, x_ref):

        self.control_rev(0.5, x_ref)
        self.arm1 = True
        self.arm2 = False
        self.arm3 = False
        self.isExtended = False
        self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.arm_controller()
        # work claw when armp is at top peg position
        # scored = True when claw released

    def mobilize(self, x_ref):
        self.control_rev(0.5, x_ref)

        self.arm3 = True
        self.arm1 = False
        self.arm2 = False

        self.isExtended = False
        self.doubles.set(DoubleSolenoid.Value.kReverse)

        self.arm_controller()

        if self.tlm.getPosition() <= (-(195 - 38) / self.wheel_circ) * self.rev:
            self.mobility = True

    def balancing(self):
        self.balanced = True
        pitch = self.navx.getPitch()
        self.mec_drive.driveCartesian(-pitch * 0.05, 0, 0)

    def control_rev(self, power, x):
        self.bottom_left.set(power * (((x * self.rev * (1 / self.wheel_circ)) - self.blm.getPosition()) / (
                    x * self.rev * (1 / self.wheel_circ))))
        self.top_left.set(power * (((x * self.rev * (1 / self.wheel_circ)) - self.tlm.getPosition()) / (
                    x * self.rev * (1 / self.wheel_circ))))
        self.bottom_right.set(power * (((x * self.rev * (1 / self.wheel_circ)) - self.brm.getPosition()) / (
                    x * self.rev * (1 / self.wheel_circ))))
        self.top_right.set(power * (((x * self.rev * (1 / self.wheel_circ)) - self.trm.getPosition()) / (
                    x * self.rev * (1 / self.wheel_circ))))
        # self.mec_drive.driveCartesian(-power*(((x*self.rev*(1/self.wheel_circ))-self.trm.getPosition())/(
        # x*self.rev*(1/self.wheel_circ))), joy_x * 0.5, joy_z * 0.5) 

    def arm_orientation(self, power, x, r):
        self.arm.set(power * ((r - x) / r))

    def arm_controller(self):
        if self.arm1:
            self.arm_orientation(0.5, self.arm_encoder.getPosition(), self.preSet_topPeg)
        elif self.arm2:
            self.arm_orientation(0.5, self.arm_encoder.getPosition(), self.preSet_middlePeg)
        elif self.arm3 and self.arm_encoder.getPosition() >= 15:
            # self.arm_orientation(0.005,self.arm_encoder.getPosition(),12)
            self.arm.set(0)
        elif self.arm3 and (15 >= self.arm_encoder.getPosition() >= 10):
            # self.arm_orientation(0.005,self.arm_encoder.getPosition(),12)
            self.arm.set(0.0005)
        elif self.arm3 and (10 >= self.arm_encoder.getPosition() >= 4):
            # self.arm_orientation(0.005,self.arm_encoder.getPosition(),12)
            self.arm.set(0.005)
        else:
            self.arm.set(0)
        # self.arm.set(power*((r-x)+power_2*(dx))/r)

            


if __name__ == "__main__":
    wpilib.run(MyRobot)

# to upload code, type and enter this in the terminal
# py -3 robot.py deploy --skip-tests
