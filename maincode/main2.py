#!/usr/bin/env python3
import math

import wpilib
from wpilib import PneumaticsModuleType, Compressor, Joystick, DoubleSolenoid, drive,interfaces, DigitalInput
from rev import CANSparkMax, RelativeEncoder, CANSparkMaxLowLevel
import numpy as np
scored,mobility,balanced = False,False,False;

class Robot:
    # Wheels
    wheel_radius =  3 * (0.0254);	 	  	 	 	# Radius of the wheel
    wheel_mass = 0.5*(1/2.20462); 	 	  	 	 	# mass of the wheel
    I_wheel = 1/2*wheel_mass*wheel_radius**2			# Moment of Inertia of the robot wheels
    wheel_friction = 0.1;				 	  	 	# Damping coefficient of the robot wheels

    # Robot Chasis
    base_length = 27 * (0.0254); 	 	 # Length of the robot (INCORRECT NEED TO COMPUTE)
    base_width = 24 * (0.0254); 		 	 # width of the robot  (INCORRECT NEED TO COMPUTE)
    base_height = 4 * (0.0254);	 		 # height of the chassis  (INCORRECT NEED TO COMPUTE)
    robot_mass = 125 * (1/2.20462);  	 # Mass of the robot in Kg
    robot_height = 36 *(0.0254);	 	 	 # height of the robot (wheels not included)  (INCORRECT NEED TO COMPUTE)

    # Arm
    arm_length_min = 25 * (0.0254); 		 # minimum arm length
    arm_length_max = 49 * (0.0254);		 # maximum arm length
    Total_arm_mass = 12 * (0.453592); 	 # mass of the arm   (INCORRECT NEED TO COMPUTE)
    Extended_arm_mass = 4 * (0.453592);	 # mass of the arm   (INCORRECT NEED TO COMPUTE)
    arm_friction = 0.5;					 # rotational friction in the arm
    arm_width = 3 * (0.0254); 			 # width of the arm
    rotate_arm_width = 2 * (0.0254); 	 # width of the rotating arm
    slide_friction = 5;
    I_arm = 0.3; 						 # Inertia of the arm (INCORRECT NEED TO COMPUTE)
    pistonForce = (60*math.pi*(1.5/2)**2)*4.4482189159; # Force of the piston
    maxArmTorque = 166.4;
    # Roller Claw
    claw_mass = 10 * (0.453592); 		 # Mass of the roller claw
    claw_length = 12 * (0.0254);		 	 # Length of the claw
    I_claw = 0.01; 						 # Moment of inertia of the claw (INCORRECT NEED TO COMPUTE)

def scoring_Controller(x,xDot,arm_theta,arm_thetaDot,l1,l1Dot):
    global scored;
    # To score we must drive forward, position the arm, and release the cube on the top level
    x_ref = 0; # this is the target position to score (DRIVE ROBOT TO THIS POINT)
    arm_theta_ref = 0; # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
    arm_length_ref = Robot.arm_length_min; # this is the target arm length

    K_drive = -0.3; # this is the control gain for driving the robot during scroring
    K_speed = 0.4; # this is the control gain for the robots speed.
    K_arm_orient = 1.5; # this is the control gain for the robot arm orientation
    K_arm_ang_vel = 0.5; # this is the control gain for the robot's arm angulat velocity
    U = np.zeros((6,1)); # Predefine the robot's control input as a vector

    # compute input
    ref = np.array([x_ref,0,arm_theta_ref,0,arm_length_ref,0]); # vector of references
    X = np.array([x,xDot,arm_theta,arm_thetaDot,l1,l1Dot]); # vector of actual states
    err = ref-X;
    #need to make sure to set max arm length after arm is at certain angle
    #if arm theta ref within a certain range?


    U[0] = K_drive*err[0] + K_speed*err[1];
    U[1] = -U[0];
    U[2] = K_arm_orient*err[2] + K_arm_ang_vel*err[3];

    if(arm_theta <= 5*(math.pi)/180):
        ref[4]=Robot.arm_length_max
        #pdb.set_trace()
    if(ref[4]==Robot.arm_length_max):
        U[3] = Robot.pistonForce;
    elif(ref[4]==Robot.arm_length_min):
        U[3] = -Robot.pistonForce;


    # Check if robot has scored (ARE THE STATES EQUAL TO THE REFERENCE VALUES)
    if(np.abs(err[0])<=0.065 and np.abs(err[1])<=0.15):
        scored = True;
    # 		pdb.set_trace();
    # 	pdb.set_trace();
    return U

def mobility_Controller(x,xDot,arm_theta,arm_thetaDot,l1,l1Dot):

    # To get mobility points we must drive backwards over the charging station and leave the community
    x_ref = (96.75+24)*(0.0254); # this is the target position to score (DRIVE ROBOT TO THIS POINT)
    arm_theta_ref = 70*(math.pi/180); # this is the target arm orientation (DRIVE THE ARM TO THIS ORIENTATION)
    arm_length_ref = Robot.arm_length_min; # this is the target arm length

    K_drive = -1.5; # this is the control gain for driving the robot during scroring
    K_speed = 0.6; # this is the control gain for the robots speed.
    K_arm_orient = 1; # this is the control gain for the robot arm orientation
    K_arm_ang_vel = 0.5; # this is the control gain for the robot's arm angulat velocity

    U = np.zeros((6,1)); # Predefine the robot's control input as a vector
    # compute input
    ref = np.array([x_ref,0,arm_theta_ref,0,arm_length_ref,0]); # vector of references
    X = np.array([x,xDot,arm_theta,arm_thetaDot,l1,l1Dot]); # vector of actual states
    err = ref-X;

    U[0] = K_drive*err[0] + K_speed*err[1];
    U[1] = -U[0];
    U[2] = K_arm_orient*err[2] + K_arm_ang_vel*err[3];

    # 	pdb.set_trace()
    if(ref[4]==Robot.arm_length_max):
        U[3] = Robot.pistonForce;
    elif(ref[4]==Robot.arm_length_min):
        U[3] = -Robot.pistonForce;


    if(np.abs(err[0])<=0.098*x_ref and np.abs(err[1])<=0.005):
        mobility = True;
    # sim2.bal(98,56,110)


    return U
def Inputs(x,xDot,arm_theta,arm_thetaDot,l1,l1Dot):
    #x is robots position (encoder)
    #xDot velocity
    #arm_theta is arm angle
    #arm_thetaDot is angular velocity
    #l1 is arm length(1 of 2 when button is pressed)
    #l1dot is how fast arm length is changing
    #global scored;
    if(not scored):
        # the current priority is to score, so implement a controller for scoring
        U = scoring_Controller(x,xDot,arm_theta,arm_thetaDot,l1,l1Dot);
    elif(not mobility):
        # the current priority is to get mobility point, so implement a controller to leave the grid
        U = mobility_Controller(x,xDot,arm_theta,arm_thetaDot,l1,l1Dot);
        # 		pdb.set_trace();
    # else:
    #     # the current priority is to balance, so implement a controller for balancing
    #     U = balanced_Controller();
    #     # 		pdb.set_trace();
    #     # 	pdb.set_trace();
    if(U[2]>= Robot.maxArmTorque):
        U[2]= Robot.maxArmTorque;
    return U;

def balance(IC,time,m,dt,l_robot,w_robot,l_charge,w_charge,h_charge,KDx,KPt,KDt):
    g = -9.81; # gravity
    Cd = 50;
    # Pre-define arrays
    x = np.zeros(len(time));
    xDot = np.zeros(len(time));
    theta = np.zeros(len(time));
    thetaDot = np.zeros(len(time));
    u = np.zeros(len(time));

    # Controller
    KPx = 0;
    #   KDx = -60;
    #   KPt = 15;
    #   KDt = 95;
    #pdb.set_trace()

class MyRobot(wpilib.TimedRobot):
    """Main robot class."""
    preSet_middlePeg = 1 #confirm later
    preSet_topPeg = 1.5 # confirm later

    isExtended = False

    def robotInit(self):


        wpilib.CameraServer.launch('vision.py:main')
        self.joystick = Joystick(0)
        self.controller = interfaces.GenericHID(0)
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.arm = CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless)

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
        self.wheel_circ = 2*math.pi*4

        self.doubles = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)
        self.rev = 6.78
        #self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.arm_encoder.setPosition(0)


    def autonomousInit(self):
        """Called only at the beginning of autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode."""
        x = self.tlm.getPosition();
        dx = self.tlm.getVelocity();
        arm_theta = self.arm_encoder.getPosition();
        arm_thetaDot = self.arm_encoder.getVelocity();
        l1 = self.isExtended;
        xDot = self.tlm.getVelocity()

        U = Inputs(x,xDot,arm_theta,arm_thetaDot,l1,0)
        self.bottom_left.set(U[0]/12)
        self.top_left.set(U[0]/12)
        self.bottom_right.set(U[0]/12)
        self.top_right.set(U[0]/12)
        self.arm.set(U[2]/12)
        if(U[3]>=0):
            self.doubles.set(DoubleSolenoid.Value.kForward)
        else:
            self.doubles.set(DoubleSolenoid.Value.kReverse)
        #U = Inputs(0, 0, 0, 0, 0, 0)
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
        self.doubles.set(DoubleSolenoid.Value.kReverse)
        self.tlm.setPosition(0.0)
        self.blm.setPosition(0.0)
        self.trm.setPosition(0.0)
        self.brm.setPosition(0.0)
        self.arm_encoder.setPosition(0)

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
    def arm_orientation(self,power,power_2,x,dx,r):
        self.arm.set(power*((r-x)+power_2*(dx))/r)

    def teleopPeriodic(self):
        """Called every 20ms in teleoperated mode"""

        if self.controller.getRawButton(5):
            self.isExtended= True
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

        if math.fabs(self.joystick.getY())<0.3:
            joy_y = 0
        if math.fabs(self.joystick.getX())<0.3:
            joy_x = 0
        if math.fabs(self.joystick.getZ())<0.3:
            joy_z = 0
        self.mec_drive.driveCartesian(joy_y * -0.5, joy_x * 0.5, joy_z * 0.5) #cant be on at same time as controll_rev()


        self.logger.info(self.arm_encoder.getPosition())


        # Move a motor with a Joystick
        #self.controll_rev(0.3,2*math.pi*4)
        # if self.controller.getRawButton(3):
        #     self.arm_orientation(0.5,0.1,self.arm_encoder.getPosition(),self.arm_encoder.getVelocity(),self.preSet_topPeg)
        # if self.controller.getRawButton(4):
        #     self.arm_orientation(0.5,0.1,self.arm_encoder.getPosition(),self.arm_encoder.getVelocity(),self.preSet_middlePeg)
        # if self.controller.getRawButton(5):
        #     self.arm_orientation(0.5,0.1,self.arm_encoder.getPosition(),self.arm_encoder.getVelocity(),0)


        #self.tlm.setPosition(0)
        #6.78
        #self.logger.info(self.tlm.getPosition())
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
