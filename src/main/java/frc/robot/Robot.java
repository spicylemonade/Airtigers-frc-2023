// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot 
{

	Joystick joystick = new Joystick(0);
	PS4Controller ps4 = new PS4Controller(1);
	Timer time = new Timer();

	CANSparkMax frontLeftMotor = new CANSparkMax(1,MotorType.kBrushless); 
	CANSparkMax backLeftMotor = new CANSparkMax(2, MotorType.kBrushless); 
	CANSparkMax frontRightMotor = new CANSparkMax(4, MotorType.kBrushless); 
	CANSparkMax backRightMotor = new CANSparkMax(3, MotorType.kBrushless);
	MecanumDrive drivetrain = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	CANSparkMax armJoint = new CANSparkMax(5, MotorType.kBrushless);
	CANSparkMax wristJoint = new CANSparkMax(6, MotorType.kBrushless);
	CANSparkMax claw = new CANSparkMax(7, MotorType.kBrushless);
	CANSparkMax leftIntake = new CANSparkMax(8, MotorType.kBrushless);
	CANSparkMax rightIntake = new CANSparkMax(9, MotorType.kBrushless);
	DifferentialDrive intake = new DifferentialDrive(leftIntake, rightIntake);
	
	Solenoid armExtend = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
	Solenoid armRetract = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
	Boolean setPWM = true;

	@Override
	public void robotInit() 
	{
	frontRightMotor.setInverted(true);
	backRightMotor.setInverted(true);
	frontLeftMotor.getEncoder().setPosition(0.0);
	backLeftMotor.getEncoder().setPosition(0.0);
	frontRightMotor.getEncoder().setPosition(0.0);
	backRightMotor.getEncoder().setPosition(0.0);
	}

	@Override
	public void teleopInit()
	{
	
	}
	
	@Override
	public void teleopPeriodic() 
	{
		drivetrain.driveCartesian(Helper.scaleInput(joystick.getY()), Helper.scaleInput(joystick.getX()), Helper.scaleInput(joystick.getZ()));
		armJoint.set(Helper.scaleInput(ps4.getLeftY()));
		wristJoint.set(Helper.scaleInput(ps4.getRightY()));
		claw.set(Helper.scaleInput(ps4.getRightX()));

		if (setPWM) {
			Helper.solenoidPWM(armExtend, ps4.getLeftX());
			Helper.solenoidPWM(armRetract, -ps4.getLeftX());
		}
		setPWM=!setPWM;

		double lr2 = Helper.scaleInput(ps4.getL2Axis())-Helper.scaleInput(ps4.getR2Axis());
		if (ps4.getL1Button()&&ps4.getR1Button()) {
			intake.arcadeDrive(0, lr2, false);
		}
		else if (ps4.getL1Button()) {
			intake.arcadeDrive(lr2, -1, false);
		}
		else if (ps4.getL2Button()) {
			intake.arcadeDrive(lr2, 1, false);
		}
		else {
			intake.arcadeDrive(lr2, 0, false);
		}
	}

	@Override
	public void autonomousInit()
	{
	time.reset();
	time.start();
	} 

	@Override
	public void autonomousPeriodic()
	{
	
	}

	@Override
	public void testPeriodic()
	{

	}
}
