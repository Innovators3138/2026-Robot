package frc.robot.harnesses;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RobotRelativeTestRobot extends LoggedRobot {

  private RobotRelativeTestContainer robotContainer;

  public RobotRelativeTestRobot() {
    robotContainer = new RobotRelativeTestContainer();
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("Project", "2026-Robot");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/U/logs")); // USB stick
      Logger.addDataReceiver(new NT4Publisher()); // live view
    } else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
