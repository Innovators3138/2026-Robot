package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommands {
  public static Pose2d BLUE_TARGET = new Pose2d(2.5, 4.3, Rotation2d.kZero);
  public static Pose2d RED_TARGET = new Pose2d(14, 4.37278, Rotation2d.k180deg);

  public static Pose2d getHub() {
    var basePosition = RED_TARGET;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      basePosition = BLUE_TARGET;
    }
    return basePosition;
  }

  public static Command firstAuto(
      SwerveSubsystem swerveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogSubsystem) {
    var driveCommand =
        swerveSubsystem.drivetoPose(
            getHub(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));
    return driveCommand
        .alongWith(FireCommand.targetLock(shooterSubsystem, swerveSubsystem))
        .until(() -> driveCommand.isFinished())
        .andThen(FireCommand.fire(feederSubsystem, hotdogSubsystem));
  }
}
