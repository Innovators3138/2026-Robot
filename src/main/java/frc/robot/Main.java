package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.harnesses.SwerveTestRobot;
import frc.robot.harnesses.VisionTestRobot;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(VisionTestRobot::new);
  }
}
