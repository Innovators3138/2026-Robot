package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  public static Pose2d RED_COLLECTION_ZONE = new Pose2d(9.0, 4.3, Rotation2d.kCW_90deg);
  public static Pose2d BLUE_COLLECTION_ZONE = new Pose2d(6.71, 4.3, Rotation2d.kCCW_90deg);

  public static Pose2d getLaunchPose() {
    var basePosition = RED_TARGET;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      basePosition = BLUE_TARGET;
    }
    return basePosition;
  }

  public static Pose2d getCollectionPose() {
    var collectionZone = RED_COLLECTION_ZONE;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      collectionZone = BLUE_COLLECTION_ZONE;
    }
    return collectionZone;
  }

  public static Command basicAuto(
      SwerveSubsystem swerveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogSubsystem) {
    var driveToClimb =
        swerveSubsystem.drivetoPose(
            new Pose2d(15.4, 5.25, Rotation2d.fromDegrees(180)),
            MetersPerSecond.of(2),
            MetersPerSecondPerSecond.of(2));
    var driveToCollectBalls =
        swerveSubsystem.drivetoPose(
            getCollectionPose(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));
    var driveToLaunchPosition =
        swerveSubsystem.drivetoPose(
            getLaunchPose(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));
    return driveToCollectBalls
        .andThen(driveToLaunchPosition)
        .alongWith(FireCommand.targetLock(shooterSubsystem, swerveSubsystem))
        .until(() -> driveToLaunchPosition.isFinished())
        .andThen(FireCommand.fire(feederSubsystem, hotdogSubsystem).withTimeout(8))
        .andThen(driveToClimb);
  }

  public static Command pathfindToPathAuto(
      SwerveSubsystem swerveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogSubsystem) {
    try {
      PathConstraints constraints =
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

      var pathplannerAuto = PathPlannerPath.fromPathFile("MakeItOutOfAllianceZoneInAuto");
      var driveToLaunchPosition =
          swerveSubsystem.drivetoPose(
              getLaunchPose(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));
      var driveToClimb =
          swerveSubsystem.drivetoPose(
              new Pose2d(15.4, 5.25, Rotation2d.fromDegrees(180)),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
      return AutoBuilder.pathfindThenFollowPath(pathplannerAuto, constraints)
          .andThen(driveToLaunchPosition)
          .alongWith(FireCommand.targetLock(shooterSubsystem, swerveSubsystem))
          .until(() -> driveToLaunchPosition.isFinished())
          .andThen(FireCommand.fire(feederSubsystem, hotdogSubsystem).withTimeout(8))
          .andThen(driveToClimb);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path for auto", e);
    }
  }
}
