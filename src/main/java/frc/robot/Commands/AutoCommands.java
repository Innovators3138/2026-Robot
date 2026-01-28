package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommands {
  private static Pose2d baselineup = new Pose2d(14, 4.07278, Rotation2d.k180deg);

  public static Command firstAuto(
      SwerveSubsystem swerveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogSubsystem) {
    return swerveSubsystem
        .drivetoPose(baselineup, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2))
        .andThen(FireCommand.targetLock(shooterSubsystem, swerveSubsystem).repeatedly())
        .alongWith(FireCommand.fire(feederSubsystem, hotdogSubsystem).repeatedly());
  }
}
