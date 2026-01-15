package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveFieldOrientated extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveInputStream inputStream;

  public DriveFieldOrientated(SwerveInputStream inputStream, SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    this.inputStream = inputStream;
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void execute() {
    swerveSubsystem.driveFieldOrientated(inputStream.get());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
