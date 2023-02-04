import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousRobot extends IterativeRobot {

  private SparkMX leftDrive = new Talon(0);
  private SparkMX rightDrive = new Talon(1);
  private SparkMX arm = new Talon(2);
  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);

  public void autonomousInit() {
    // Travel forward for about 3 1/2 feet
    leftEncoder.reset();
    rightEncoder.reset();
    leftDrive.set(0.5);
    rightDrive.set(0.5);
    while (leftEncoder.getDistance() < 42 || rightEncoder.getDistance() < 42) {
      // wait until the robot has traveled the desired distance
    }
    leftDrive.set(0);
    rightDrive.set(0);

    // Raise the arm upward to reach 3 ft 5 1/8 inches
    arm.set(0.5);
    try {
      Thread.sleep(3000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    arm.set(0);

    // Release the cube from the arm
    // (Assuming there is a mechanism in place to accomplish this)

    // Travel approximately 21.26 feet backward to the neutral zone of the field
    leftEncoder.reset();
    rightEncoder.reset();
    leftDrive.set(-0.5);
    rightDrive.set(-0.5);
    while (leftEncoder.getDistance() > -257 || rightEncoder.getDistance() > -257) {
      // wait until the robot has traveled the desired distance
    }
    leftDrive.set(0);
    rightDrive.set(0);

    // Travel approximately 18.2 feet forward to reach the charging station
    leftEncoder.reset();
    rightEncoder.reset();
    leftDrive.set(0.5);
    rightDrive.set(0.5);
    while (leftEncoder.getDistance() < 218 || rightEncoder.getDistance() < 218) {
      // wait until the robot has traveled the desired distance
    }
    leftDrive.set(0);
    rightDrive.set(0);

    // Balance onto the charging station
    // (Assuming there is a mechanism in place to accomplish this)
    // Using IMU Code
  }
}