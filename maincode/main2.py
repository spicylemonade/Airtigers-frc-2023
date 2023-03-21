#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive, interfaces, DigitalInput,PWMTalonSRX
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel
from navx import AHRS

import numpy as np
from wpimath import controller


class MyRobot(wpilib.TimedRobot):
    """Main robot class."""
    preSet_middlePeg = 25.97  # confirm later
    preSet_topPeg = 32.5  # confirm later

    isExtended = False
    scored, mobility, mobility2,balanced = False, False, False,False
    scored = True
    deg_to_count = 90*(42/360)
    arm1 = False
    arm2 = False
    arm3 = False
    wheel_circ = 2 * math.pi * 4
    rev = 6.78

    def __init__(self):
        wpilib.TimedRobot.__init__(self)
        self.joystick = Joystick(0)
        self.controller = interfaces.GenericHID(0)
        #self.navx = AHRS(wpilib.SerialPort.Port.kUSB)

        self.xbox = interfaces.GenericHID(1)
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.arm = CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless)

        self.claw_lead =  CANSparkMax(17, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.claw1 = PWMTalonSRX(7)
        self.claw2=  PWMTalonSRX(8)


        self.armlimit_bottom = DigitalInput(1)
        self.armlimit_top = DigitalInput(0)

        self.mec_drive = drive.MecanumDrive(self.top_left, self.bottom_left, self.top_right, self.bottom_right)

        self.counts2deg = 360/42
        self.counts2in = math.pi*8/6.78
        self.arm_offset_angle = 40 # degree offset of the arm
        self.tlm = self.top_left.getEncoder()
        self.blm = self.bottom_left.getEncoder()
        self.trm = self.top_right.getEncoder()
        self.brm = self.bottom_right.getEncoder()
        self.arm_encoder = self.arm.getEncoder()

        self.comp = Compressor(0, PneumaticsModuleType.CTREPCM)

        #self.comp.enableDigital()

        self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)

        self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.arm_encoder.setPosition(0)
        #self.counts2deg+self.arm_offset_angle
        self.kP = 0.5
        self.kI = 0.1
        self.kD = 0.0
        self.pid = controller.PIDController(self.kP, self.kI, self.kD)
        #self.accel = ADXL345_I2C(I2C.Port.kOnboard,20,0x53)
        #self.pitch = math.acos(9.81/self.accel.getY())


    def robotInit(self):

        wpilib.CameraServer.launch('vision.py:main')




        self.top_right.setInverted(True)
        self.bottom_left.setInverted(False)
        self.top_left.setInverted(False)
        self.bottom_right.setInverted(True)



        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        """Robot-wide initialization code should go here."""






    def autonomousInit(self):
        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        self.arm_encoder.setPosition(self.counts2deg+self.arm_offset_angle)
        """Called only at the beginning of autonomous mode."""
        # self.pid.setOutputRange(-0.5, 0.5)
        self.pid.setSetpoint(0)
        self.pid.enableContinuousInput(-180,180)
        self.navx.reset()
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""

        self.logger.info(self.navx.getPitch())
        self.logger.info(self.arm_encoder.getPosition()*self.counts2deg+self.arm_offset_angle)

        # if self.scored == False:
        #     # move forward 38 inches
        #     self.scoring()
        #     self.ref_position_inches = -188.31
        #     self.ref_leftposition = -151.24
        # elif (self.tlm.getPosition()*self.counts2in > self.ref_position_inches) and (self.mobility == False):
        #
        #     self.mec_drive.driveCartesian(-0.5*((self.ref_position_inches-self.tlm.getPosition()*self.counts2in)/self.ref_position_inches), 0, 0)
        #
        # elif (self.mobility2 == False) and not(self.tlm.getPosition() >= self.ref_leftposition):
        #     self.mobility = True
        #
        #     # self.mec_drive.driveCartesian(0, 0.3, 0) #y0.3
        #     self.mec_drive.driveCartesian(0, 0.5*((self.ref_leftposition-self.tlm.getPosition())/self.ref_leftposition), 0)
        # elif (self.tlm.getPosition() > self.ref_leftposition) and self.mobility2 == False:
        #     self.mobility2 = True
        #
        # elif (math.fabs(self.navx.getPitch())<=2 ) and self.balanced == False:
        #     # iffy rate
        #     self.mobility2 = True
        #     self.mec_drive.driveCartesian(0.3, 0, 0) #0.3
        #
        # else:
        #     self.balanced = True
        #     # self.mec_drive.driveCartesian(0.15*((self.navx.getPitch())),0,0)
        #     # self.pid.setSetpoint(0)
        #     # self.pid.setPID(self.kP, self.kI, 0.1)
        #     #self.pid.enableContinuousInput(-15,15)
        #
        #     # Check for motor stall
        #     # if self.pi:
        #     #     self.mec_drive.driveCartesian(0,0,0)
        #     # self.mec_drive.driveCartesian(self.pid.calculate(self.navx.getPitch(),0)*0.15,0,0)
        #     # if self.navx.getPitch()==0:
        #     #     self.mec_drive.driveCartesian(0,0,0)
        #
        #     # self.logger.info(self.pid.calculate(self.navx.getPitch(),0))
        #     pass

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
        # self.arm_encoder.setPosition(0)*self.counts2deg+self.arm_offset_angle

        """Called only at the beginning of teleoperated mode."""
        # self.loops = 0
        # self.timer.reset()
        # self.timer.start()
    def teleopPeriodic(self):
        """Called every 20ms in teleoperated mode"""
        #self.logger.info(self.pitch)

        if self.controller.getRawButton(5):
            self.isExtended = True
            self.doubles.set(DoubleSolenoid.Value.kForward)
        elif self.controller.getRawButton(6):
            self.isExtended = False
            self.doubles.set(DoubleSolenoid.Value.kReverse)
        else:
            self.doubles.set(DoubleSolenoid.Value.kOff)

        # self.driver()

        self.joy_y = self.joystick.getY()
        self.joy_x = self.joystick.getX()
        self.joy_z = self.joystick.getZ()
        #self.logger.info("ACCEL: " + str(self.accel.getAcceleration(ADXL345_I2C.Axes.kAxis_Z)))

        if math.fabs(self.joystick.getY()) < 0.35:
            self.joy_y = 0
        if math.fabs(self.joystick.getX()) < 0.35:
            self.joy_x = 0
        if math.fabs(self.joystick.getZ()) < 0.35:
            self.joy_z = 0
        self.mec_drive.driveCartesian(self.joy_y * -0.7, self.joy_x * 0.7,self.joy_z * 0.7)  # cant be on at same time as controll_rev()
        #self.balancing()
        #self.clawer()

        #arm
        if not self.controller.getRawButton(1):
            self.arm_controller()
        else:
            if not self.arm_encoder.getPosition()*self.counts2deg+self.arm_offset_angle >= self.preSet_middlePeg:
                self.arm.set(0.5)
                self.arm3 = True
                self.arm1 = False
                self.arm2 = False


        #claw
        #self.clawer()

        # Move a motor with a Joystick
        # self.controll_rev(0.3,2*math.pi*4)

        self.logger.info(self.navx.getPitch())
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



    def scoring(self):

        #self.control_rev(1,10)
        self.arm1 = True
        self.arm2 = False
        self.arm3 = False
        self.isExtended = True
        self.doubles.set(DoubleSolenoid.Value.kForward)
        #self.arm_controller()
        # work claw when armp is at top peg position
        # scored = True when claw released
        if self.arm_encoder.getPosition()*self.counts2deg+self.arm_offset_angle >=self.preSet_middlePeg:
            self.claw_lead.set(0.5 * ((6 - self.claw_lead.getEncoder().getPosition()) / 6))
        if self.claw_lead.getEncoder().getPosition() >= self.deg_to_count:
            self.claw1.set(0.5 * ((self.deg_to_count - self.claw1.getEncoder().getPosition()) / self.deg_to_count))
        if self.claw1.getEncoder().getPosition() >= 6:
            self.scored = True
            self.arm1 = False
            self.arm2 = False
            self.arm3 = True
            self.isExtended = False
            self.doubles.set(DoubleSolenoid.Value.kReverse)





    def control_rev(self,power,x):
        self.mec_drive.driveCartesian(0.15*power,0,0)

    # def control_rev(self,power,x):
    #     self.bottom_left.set(power*(((x*self.rev*(1/self.wheel_circ))-self.blm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
    #     self.top_left.set(power*(((x*self.rev*(1/self.wheel_circ))-self.tlm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
    #     self.bottom_right.set(power*(((x*self.rev*(1/self.wheel_circ))-self.brm.getPosition())/(x*self.rev*(1/self.wheel_circ))))
    #     self.top_right.set(power*(((x*self.rev*(1/self.wheel_circ))-self.trm.getPosition())/(x*self.rev*(1/self.wheel_circ))))

    # self.mec_drive.driveCartesian(-power*(((x*self.rev*(1/self.wheel_circ))-self.trm.getPosition())/(
    # x*self.rev*(1/self.wheel_circ))), joy_x * 0.5, joy_z * 0.5)

    def arm_orientation(self, power, x, r):
        self.arm.set(power * ((r - x) / r))

    @staticmethod
    def limit(n, minn, maxn):
        return max(min(maxn, n), minn)

    def arm_controller(self):
        if self.arm1:
            self.arm_orientation(0.7, self.arm_encoder.getPosition(), self.preSet_topPeg)
        elif self.arm2:
            self.arm_orientation(0.7, self.arm_encoder.getPosition(), self.preSet_middlePeg)
        elif self.arm3:
            self.arm.set(0.000001)
        else:
            self.arm.set(0)
        # self.arm.set(power*((r-x)+power_2*(dx))/r)
    def clawer(self):

        if self.xbox.getRawButton(4):
            self.claw_lead.set(-0.15)
        elif self.xbox.getRawButton(5):
            self.claw_lead.set(0.15)
        else:
            self.claw_lead.set(0)
        left_axis=self.xbox.getRawAxis(2)
        #self.claw_lead.set(0.5 * ((6 - self.claw_lead.getEncoder().getPosition()) / 6))
        #left_axis = 0 if left_axis<= 0.1 else left_axis
        #right_axis=self.xbox.getRawAxis(3)
        #right_axis = 0 if right_axis<= 0.1 else right_axis
        #self.claw1.set(left_axis)





if __name__ == "__main__":
    wpilib.run(MyRobot)

# to upload code, type and enter this in the terminal
# py -3 robot.py deploy --skip-tests
