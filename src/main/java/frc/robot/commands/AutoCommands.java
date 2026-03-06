package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoCommands {
  public static Pose2d BLUE_TARGET = new Pose2d(2.5, 4.3, Rotation2d.kZero);
  public static Pose2d RED_TARGET = new Pose2d(14, 4.37278, Rotation2d.k180deg);
  public static Pose2d RED_COLLECTION_ZONE = new Pose2d(9.0, 4.3, Rotation2d.kCW_90deg);
  public static Pose2d BLUE_COLLECTION_ZONE = new Pose2d(6.71, 4.3, Rotation2d.kCCW_90deg);
  public static Pose2d RED_CLIMB_POSE = new Pose2d(15.4, 5.25, Rotation2d.fromDegrees(180));
  public static Pose2d BLUE_CLIMB_POSE = new Pose2d(0.9, 2.7, Rotation2d.fromDegrees(0));
  public static SendableChooser<String> climbChooser = new SendableChooser<>();
  public static SendableChooser<String> intakeChooser = new SendableChooser<>();

  static {
    climbChooser.setDefaultOption("Left Climb", "Left Climb");
    climbChooser.addOption("Middle Climb", "Middle Climb");
    climbChooser.addOption("Right Climb", "Right Climb");
    intakeChooser.addOption("Left Intake", "Left Intake");
    intakeChooser.addOption("Depot Intake", "Depot Intake");
    intakeChooser.addOption("Right Intake", "Right Intake");
    SmartDashboard.putData(climbChooser);
    SmartDashboard.putData(intakeChooser);
  }

  public static Command climb(ClimberSubsystem climberSubsystem) {

    return climberSubsystem.extend().andThen(climberSubsystem.climb());
  }

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

  public static Pose2d getClimbPose() {
    var climbPosition = Constants.FieldConstants.getRightClimb();
    var selectedClimb = climbChooser.getSelected();
    if (selectedClimb == null) {
      return climbPosition;
    }
    if (selectedClimb.equals("Left Climb")) {
      return climbPosition;
    }
    if (selectedClimb.equals("Middle Climb")) {
      return climbPosition = Constants.FieldConstants.getMiddleClimb();
    }
    if (selectedClimb.equals("Right Climb")) {
      return climbPosition = Constants.FieldConstants.getLeftClimb();
    }
    return climbPosition;
  }

  public static Command createAuto(
      SwerveSubsystem swerveSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem,
      HotdogSubsystem hotdogSubsystem)
      throws FileVersionException, IOException, ParseException {
    var selectedClimb = climbChooser.getSelected();
    var selectedIntake = intakeChooser.getSelected();
    Command driveToShootingCommand;
    Command driveToLoadingCommand;
    Command driveToClimbCommand;
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    if (selectedIntake.equals("Depot Intake")) {
      driveToLoadingCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getDepot(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
    } else if (selectedIntake.equals("Left Intake")) {
      driveToLoadingCommand =
          AutoBuilder.pathfindThenFollowPath(
              PathPlannerPath.fromPathFile("Left Intake"), constraints);
    } else {
      driveToLoadingCommand =
          AutoBuilder.pathfindThenFollowPath(
              PathPlannerPath.fromPathFile("Right Intake"), constraints);
    }
    if (selectedClimb.equals("Left Climb")) {
      driveToClimbCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getLeftClimb(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
      driveToShootingCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getLeftShoot(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
    } else if (selectedClimb.equals("Middle Climb")) {
      driveToClimbCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getMiddleClimb(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
      driveToShootingCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getMiddleShoot(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));

    } else {
      driveToClimbCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getRightClimb(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
      driveToShootingCommand =
          swerveSubsystem.drivetoPose(
              Constants.FieldConstants.getRightShoot(),
              MetersPerSecond.of(2),
              MetersPerSecondPerSecond.of(2));
    }
    return driveToLoadingCommand
        .andThen(driveToShootingCommand)
        .andThen(FireCommand.targetLock(shooterSubsystem, swerveSubsystem).withTimeout(0.5))
        .andThen(FireCommand.fire(feederSubsystem, hotdogSubsystem).withTimeout(5))
        .andThen(driveToClimbCommand);
  }

  /*  public static PathPlannerPath getIntakePath(SwerveSubsystem swerveSubsystem)
        throws FileVersionException, IOException, ParseException {
      var intakePath = PathPlannerPath.fromPathFile("Left Intake");
      var selectedPath = intakeChooser.getSelected();
      if (selectedPath == null) {
        intakePath = PathPlannerPath.fromPathFile("Left Intake");
      }
      if (selectedPath.equals("Left Intake")) {
        intakePath = PathPlannerPath.fromPathFile("Left Intake");
      }
      if (selectedPath.equals("Depot Intake")) {

        intakePath =
            swerveSubsystem.drivetoPose(
                Constants.FieldConstants.getDepot(),
                MetersPerSecond.of(2),
                MetersPerSecondPerSecond.of(2));
      }

      if (selectedPath.equals("Right Intake")) {
        return intakePath = PathPlannerPath.fromPathFile("Right Intake");
      }
      return intakePath;
    }
  */
  /*public static Command pathfindToPathAuto(
      SwerveSubsystem swerveSubsystem,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogSubsystem,
      frc.robot.subsystems.ClimberSubsystem climberSubsystem) {
    try {
      PathConstraints constraints =
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

      var pathplannerAuto = getIntakePath(swerveSubsystem);
      var driveToLaunchPosition =
          swerveSubsystem.drivetoPose(
              getLaunchPose(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));
      var driveToClimbPosition =
          swerveSubsystem.drivetoPose(
              getClimbPose(), MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2));

      return AutoBuilder.pathfindThenFollowPath(pathplannerAuto, constraints)
          .andThen(driveToLaunchPosition)
          .alongWith(FireCommand.targetLock(shooterSubsystem, swerveSubsystem))
          .until(() -> driveToLaunchPosition.isFinished())
          .andThen(FireCommand.fire(feederSubsystem, hotdogSubsystem).withTimeout(8))
          .andThen(driveToClimbPosition)
          .andThen(climb(climberSubsystem));
    } catch (Exception e) {
      throw new RuntimeException("Failed to load path for auto", e);
    }
  } */
}
