package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.ShotSimulator;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private ShotSimulator shotSimulator;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousInit() {
    try {
      autonomousCommand = robotContainer.getAutonomousCommand();
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
    robotContainer.swerveSubsystem.resetOdometry(Constants.FieldConstants.getInitialPose());

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
      ;
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    shotSimulator =
        new ShotSimulator(
            robotContainer, robotContainer.feederSubsystem, robotContainer.swerveSubsystem);
    shotSimulator.generateBalls();
  }

  @Override
  public void simulationPeriodic() {
    var dt = Seconds.of(getPeriod());
    shotSimulator.update(dt);
  }
}
