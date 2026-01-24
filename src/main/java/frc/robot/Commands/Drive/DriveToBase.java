package frc.robot.Commands.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToBase extends Command {
  private final Pose2d baselineup = new Pose2d(12.873, 4.07278, Rotation2d.kZero);
  private final Pose2d basePosition = Constants.getHub();
  private final SwerveSubsystem swerveSubsystem;
  private Command driveToPose;
  PathConstraints constraints =
      new PathConstraints(2.0, 2.0, Units.degreesToRadians(180), Units.degreesToRadians(720));

  public DriveToBase(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(baselineup, constraints, 0);

    driveToPose = pathfindingCommand;
    driveToPose.initialize();
  }

  @Override
  public void execute() {
    driveToPose.execute();
  }

  @Override
  public void end(boolean interrupted) {
    if (driveToPose == null) {
      return;
    }
    driveToPose.end(interrupted);
    driveToPose = null;
  }

  @Override
  public boolean isFinished() {
    if (driveToPose == null) {
      return true;
    }
    return driveToPose.isFinished();
  }
}
