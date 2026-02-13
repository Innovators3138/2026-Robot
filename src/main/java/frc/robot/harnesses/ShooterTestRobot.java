package frc.robot.harnesses;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ShooterTestRobot extends TimedRobot {

  private ShooterTestContainer robotContainer;

  public ShooterTestRobot() {

    robotContainer = new ShooterTestContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
