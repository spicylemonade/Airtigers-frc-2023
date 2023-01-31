import java.util.ArrayList;
import java.math.*;

public class AntiTipping {
  private static final double THRESHOLD = 0.5;
  private ArrayList<Double> angleHistory = new ArrayList<Double>();
  private int historySize = 10;
  
  public void updateAngle(double angle) {
    angleHistory.add(angle);
    if (angleHistory.size() > historySize) {
      angleHistory.remove(0);
    }
  }
  
  public boolean isRobotTipping() {
    double sum = 0;
    for (Double angle : angleHistory) {
      sum += angle;
    }
    double averageAngle = sum / angleHistory.size();
    return Math.abs(averageAngle) > THRESHOLD;
  }
  
  public void preventTipping() {
    if (isRobotTipping()) {
      // Add code to prevent robot from tipping over here
    }
  }
}
