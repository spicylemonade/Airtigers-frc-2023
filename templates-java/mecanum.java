// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;

//import for PS4
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

//import for Limelight
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//import for mecanumdrive
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

//spark motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;


import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot 
{
  private MecanumDrive mecanumDrive;
  Joystick joystick = new Joystick(0);
  GenericHID mecJoystick = new GenericHID(1);
  Timer time = new Timer();
  CANSparkMax m_topLeft;
  CANSparkMax m_bottomLeft;
  CANSparkMax m_topRight;
  CANSparkMax m_bottomRight;
  RelativeEncoder tlM;
  RelativeEncoder blM;
  RelativeEncoder trM;
  RelativeEncoder brM;

  @Override
  public void robotInit() 
  {
    m_topLeft = new CANSparkMax(1,MotorType.kBrushless); 
    m_bottomLeft = new CANSparkMax(2, MotorType.kBrushless); 
    m_topRight = new CANSparkMax(4, MotorType.kBrushless); 
    m_bottomRight = new CANSparkMax(3, MotorType.kBrushless);

    m_topRight.setInverted(true);
    m_bottomRight.setInverted(true);
    m_topLeft.setInverted(false);
    m_bottomLeft.setInverted(false);

    //CameraServer.startAutomaticCapture();

    mecanumDrive = new MecanumDrive(m_topLeft, m_bottomLeft, m_topRight, m_bottomRight);

    tlM = m_topLeft.getEncoder();
    blM = m_bottomLeft.getEncoder();
    trM = m_topRight.getEncoder();
    brM = m_bottomRight.getEncoder();
    
    // setting the initial positions of the wheels
    tlM.setPosition(0.0);
    blM.setPosition(0.0);
    trM.setPosition(0.0);
    brM.setPosition(0.0);
  }

  @Override
  public void teleopInit()
  {
    
  }
  
  @Override
  public void teleopPeriodic() 
  {
    //getting joystick values
    double joystickY = joystick.getY();
    double joystickX = joystick.getX();
    double joystickZ = joystick.getZ();
    double xyspeed = 0.4; //correct is 0.6
    double zspeed = 0.3; //correct is 0.3

    //deadband values
    if (Math.abs(joystickY) < 0.3)
    {
      joystickY = 0.0;
    }
    if (Math.abs(joystickX) < 0.3)
    {
      joystickX = 0.0;
    }
    if (Math.abs(joystickZ) < 0.3)
    {
      joystickZ = 0.0;
    }

    //mecanum drive function
    mecanumDrive.driveCartesian(-joystickY*xyspeed, joystickX*xyspeed, joystickZ*zspeed);

    //position
    SmartDashboard.putNumber("TopLMotorPos", tlM.getPosition());
    SmartDashboard.putNumber("BottomLMotorPos", blM.getPosition());
    SmartDashboard.putNumber("TopRMotorPos", trM.getPosition());
    SmartDashboard.putNumber("BottomRMotorPos", brM.getPosition());
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
