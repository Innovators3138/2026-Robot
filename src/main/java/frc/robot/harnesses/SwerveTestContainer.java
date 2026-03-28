package frc.robot.harnesses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SwerveTestContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);
  public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

  SwerveTestContainer() {
    driverXbox.start().onTrue(swerveSubsystem.resetOdometry());
    driverXbox
        .a()
        .onTrue(swerveSubsystem.drivetoPose(new Pose2d(13.5, 4.037, Rotation2d.fromDegrees(180))));

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOriented(driverXbox, operatorXbox));
  }
}
