#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive,interfaces, DigitalInput
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel
class MyRobot(wpilib.TimedRobot):
    """Main robot class."""


    def robotInit(self):
        self.joystick = Joystick(0)
        self.controller = interfaces.GenericHID(0)
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.arm = CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless)

        self.armlimit = DigitalInput(0)


        self.top_right.setInverted(True)
        self.bottom_left.setInverted(False)
        self.top_left.setInverted(False)
        self.bottom_right.setInverted(True)

        self.mec_drive = drive.MecanumDrive(self.top_left, self.bottom_left, self.top_right, self.bottom_right)


        self.tlm = self.top_left.getEncoder()
        self.blm = self.bottom_left.getEncoder()
        self.trm = self.top_right.getEncoder()
        self.brm = self.bottom_right.getEncoder()

        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        """Robot-wide initialization code should go here."""
        self.comp = Compressor(0, PneumaticsModuleType.CTREPCM)
        #self.comp.enableDigital()
        self.wheel_circ = 2*math.pi*4

        self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)
        self.rev = 6.78


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
        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)

        """Called only at the beginning of teleoperated mode."""
        # self.loops = 0
        # self.timer.reset()
        # self.timer.start()
    # def controll_rev(self,power,x):
    #     self.bottom_left.set(power*(((x*self.rev)-self.blm.getPosition())/(x*self.rev)))
    #     self.top_left.set(power*(((x*self.rev)-self.tlm.getPosition())/(x*self.rev)))
    #     self.bottom_right.set(power*(((x*self.rev)-self.brm.getPosition())/(x*self.rev)))
    #     self.top_right.set(power*(((x*self.rev)-self.trm.getPosition())/(x*self.rev)))
    def controll_rev(self,power,x):
        self.bottom_left.set(power*(((x*self.rev*(1/self.wheel_circ))-self.blm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
        self.top_left.set(power*(((x*self.rev*(1/self.wheel_circ))-self.tlm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
        self.bottom_right.set(power*(((x*self.rev*(1/self.wheel_circ))-self.brm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
        self.top_right.set(power*(((x*self.rev*(1/self.wheel_circ))-self.trm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
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
        self.mec_drive.driveCartesian(joy_y * -0.5, joy_x * 0.5, joy_z * 0.5)


        if self.controller.getRawButton(3):
            self.arm.set(0.5)
        elif self.controller.getRawButton(4):
            self.arm.set(-0.5)


        # Move a motor with a Joystick
        #self.controll_rev(0.3,2*math.pi*4)

        self.logger.info(self.tlm.getPosition())
        if(self.armlimit.get()):
            self.arm.set(0)

        #self.tlm.setPosition(0)
        #6.78
        # self.motor.set(self.lstick.getY())
        #
        # # Print out the number of loop iterations passed every second
        # self.loops += 1

        # if self.timer.advanceIfElapsed(1):
        #     self.logger.info("%d loops / second", self.loops)
        #     self.loops = 0


if __name__ == "__main__":
    wpilib.run(MyRobot)

# to upload code, type and enter this in the terminal
# py -3 robot.py deploy --skip-tests
