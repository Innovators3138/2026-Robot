package frc.robot.harnesses;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveTestRobot extends TimedRobot {

  private SwerveTestContainer robotContainer;

  public SwerveTestRobot() {
    robotContainer = new SwerveTestContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
