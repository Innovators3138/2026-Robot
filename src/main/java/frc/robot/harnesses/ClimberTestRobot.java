package frc.robot.harnesses;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ClimberTestRobot extends TimedRobot {
  private ClimberTestContainer robotContainer;

  public ClimberTestRobot() {
    robotContainer = new ClimberTestContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
