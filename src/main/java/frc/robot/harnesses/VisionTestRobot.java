package frc.robot.harnesses;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class VisionTestRobot extends TimedRobot {

  private VisionTestContainer robotContainer;

  public VisionTestRobot() {
    robotContainer = new VisionTestContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
