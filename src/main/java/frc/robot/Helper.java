package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Helper {
	public static double scaleInput(double input){
		return input*input; //Change this to adjust the way controller inputs are scaled
	}
	public static void solenoidPWM(Solenoid solenoid, double input) {
		if (input>0.8) {
			solenoid.set(true);
		}
		else if (input>0.6) {
			solenoid.set(false);
			solenoid.setPulseDuration(0.03);
			solenoid.startPulse();
		}
		else if (input>0.4) {
			solenoid.set(false);
			solenoid.setPulseDuration(0.02);
			solenoid.startPulse();
		}
		else if (input>0.2) {
			solenoid.set(false);
			solenoid.setPulseDuration(0.01);
			solenoid.startPulse();
		}
		else{
			solenoid.set(false);
		}
	}
}