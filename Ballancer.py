import wpilib
import wpilib.drive
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
        # Set up NavX sensor
        self.navx = navx.AHRS.create_spi()

        # Set up mecanum drive train
        self.frontLeftMotor = wpilib.Spark(0)
        self.rearLeftMotor = wpilib.Spark(1)
        self.frontRightMotor = wpilib.Spark(2)
        self.rearRightMotor = wpilib.Spark(3)
        self.drive = wpilib.drive.MecanumDrive(self.frontLeftMotor, self.rearLeftMotor, self.frontRightMotor, self.rearRightMotor)
        self.drive.setExpiration(0.1)

        # Set up network tables
        self.nt = NetworkTables.getTable("SmartDashboard")

        # Set up PID controller for balancing
        self.kP = 0.5
        self.kI = 0.1
        self.kD = 0.0
        self.kF = 0.0
        self.pid = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.navx, output=self)

        # Set up motor speed limits
        self.maxSpeed = 0.5
        self.minSpeed = -0.5

    def autonomousInit(self):
        # Start PID controller
        self.pid.setOutputRange(self.minSpeed, self.maxSpeed)
        self.pid.setSetpoint(0)
        self.pid.enable()

    def autonomousPeriodic(self):
        # Update PID controller
        self.pid.setSetpoint(0)
        self.pid.setPID(self.kP, self.kI, self.kD, self.kF)
        self.pid.enable()

        # Check for motor stall
        if self.pid.getOutput() == 0:
            self.drive.stopMotor()

    def disabledInit(self):
        # Stop PID controller
        self.pid.disable()

    def pidWrite(self, output):
        # Set motor speeds
        self.drive.driveCartesian(0, output, 0)

if __name__ == "__main__":
    wpilib.run(MyRobot)
