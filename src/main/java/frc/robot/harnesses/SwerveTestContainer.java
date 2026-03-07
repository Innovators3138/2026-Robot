package frc.robot.harnesses;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTestContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  SwerveTestContainer() {
    driverXbox.start().onTrue(swerveSubsystem.resetOdometry());
    driverXbox
        .a()
        .onTrue(
            swerveSubsystem
                .resetOdometry()
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(2, 1, Rotation2d.kZero),
                        MetersPerSecond.of(0.5),
                        MetersPerSecondPerSecond.of(0.5)))
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(2, 2, Rotation2d.kZero),
                        MetersPerSecond.of(0.5),
                        MetersPerSecondPerSecond.of(0.5)))
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(1, 2, Rotation2d.kZero),
                        MetersPerSecond.of(0.5),
                        MetersPerSecondPerSecond.of(0.5)))
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(1, 1, Rotation2d.kZero),
                        MetersPerSecond.of(0.5),
                        MetersPerSecondPerSecond.of(0.5)))
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(1, 1, Rotation2d.kZero),
                        MetersPerSecond.of(0.5),
                        MetersPerSecondPerSecond.of(0.5))));
    driverXbox
        .b()
        .onTrue(
            swerveSubsystem
                .resetOdometry()
                .andThen(
                    swerveSubsystem.drivetoPose(
                        new Pose2d(2, 2, Rotation2d.k180deg),
                        MetersPerSecond.of(1),
                        MetersPerSecondPerSecond.of(1))));
    driverXbox.povLeft().onTrue(swerveSubsystem.sysIdDriveMotorCommand());

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOriented(driverXbox, operatorXbox));
  }
}
