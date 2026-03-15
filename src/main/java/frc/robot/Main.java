package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.harnesses.ShooterTestRobot;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(ShooterTestRobot::new);
  }
}
