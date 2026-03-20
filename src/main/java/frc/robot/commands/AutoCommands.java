package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoCommands {
  public static Pose2d BLUE_TARGET = new Pose2d(2.5, 4.3, Rotation2d.kZero);
  public static Pose2d RED_TARGET = new Pose2d(14, 4.37278, Rotation2d.k180deg);
  public static Pose2d RED_COLLECTION_ZONE = new Pose2d(9.0, 4.3, Rotation2d.kCW_90deg);
  public static Pose2d BLUE_COLLECTION_ZONE = new Pose2d(6.71, 4.3, Rotation2d.kCCW_90deg);
  public static Pose2d RED_CLIMB_POSE = new Pose2d(15.4, 5.25, Rotation2d.fromDegrees(180));
  public static Pose2d BLUE_CLIMB_POSE = new Pose2d(0.9, 2.7, Rotation2d.fromDegrees(0));
  public static SendableChooser<String> startingChooser = new SendableChooser<>();
  public static SendableChooser<String> intakeChooser = new SendableChooser<>();
  public double axisMultiplier;
  public static Pose2d BLUE_MIDDLE_STARTING_POSE = new Pose2d(3.509, 4, Rotation2d.k180deg);
  public static Pose2d RED_MIDDLE_STARTING_POSE = new Pose2d(12.706, 4.1, Rotation2d.kZero);

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

  public static Pose2d getMiddleStartingPose() {
    var basePosition = RED_MIDDLE_STARTING_POSE;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      basePosition = BLUE_MIDDLE_STARTING_POSE;
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
    var selectedStart = startingChooser.getSelected();
    if (selectedStart == null) {
      return climbPosition;
    }
    if (selectedStart.equals("Right Start")) {
      return climbPosition;
    }
    if (selectedStart.equals("Middle Start")) {
      return climbPosition = Constants.FieldConstants.getMiddleClimb();
    }
    if (selectedStart.equals("Left Start")) {

      return climbPosition = Constants.FieldConstants.getLeftClimb();
    }
    return climbPosition;
  }

  private static Pose2d getStartingPosition() {
    var selectedStart = startingChooser.getSelected();
    if (selectedStart.equals("Right Start")) {

      return Constants.FieldConstants.getRightStart();
    } else if (selectedStart.equals("Middle Start")) {

      return getMiddleStartingPose();
    } else {

      return Constants.FieldConstants.getLeftStart();
    }
  }

  private static Command getShootingCommand(RobotContainer robotContainer) {
    var selectedStart = startingChooser.getSelected();
    if (selectedStart.equals("Right Start")) {

      return robotContainer.swerveSubsystem.drivetoPose(Constants.FieldConstants.getRightShoot());
    } else if (selectedStart.equals("Middle Start")) {

      return robotContainer.swerveSubsystem.drivetoPose(Constants.FieldConstants.getMiddleShoot());

    } else {

      return robotContainer.swerveSubsystem.drivetoPose(Constants.FieldConstants.getLeftShoot());
    }
  }

  private static Command getLoadingCommand(RobotContainer robotContainer)
      throws FileVersionException, IOException, ParseException {
    var selectedIntake = intakeChooser.getSelected();
    PathConstraints constraints =
        new PathConstraints(5.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    if (selectedIntake.equals("Depot Intake")) {
      return robotContainer.swerveSubsystem.drivetoPose(Constants.FieldConstants.getDepot());
    } else if (selectedIntake.equals("Left Intake")) {
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("Left Intake"), constraints);
    } else if (selectedIntake.equals("Right Intake")) {

      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("Right Intake"), constraints);
    } else {
      return Commands.none();
    }
  }

  public static Command createAuto(RobotContainer robotContainer)
      throws FileVersionException, IOException, ParseException {
    var selectedClimb = startingChooser.getSelected();
    var selectedIntake = intakeChooser.getSelected();
    Command driveToShootingCommand = getShootingCommand(robotContainer);
    Command driveToLoadingCommand = getLoadingCommand(robotContainer);

    return Commands.runOnce(() -> RobotContainer.intakeSubsystem.openHopper())
        .andThen(
            Commands.runOnce(
                () -> {
                  robotContainer.visionSubsystem.initializePose(new Pose3d(getStartingPosition()));
                  robotContainer.swerveSubsystem.resetOdometry(getStartingPosition());
                }))
        .andThen(robotContainer.intakeSubsystem.setAngularVelocity(RPM.of(900)))
        .raceWith(driveToLoadingCommand.andThen(driveToShootingCommand))
        .andThen(
            FireCommand.targetLock(robotContainer.shooterSubsystem, robotContainer.swerveSubsystem)
                .withTimeout(0.5))
        .andThen(
            FireCommand.fire(
                robotContainer.feederSubsystem,
                robotContainer.hotdogSubsystem,
                robotContainer.shooterSubsystem))
        .andThen(
            Commands.runOnce(
                () -> {
                  robotContainer.intakeSubsystem.resetServo();
                }))
        .andThen(
            Commands.runOnce(
                () -> {
                  robotContainer.visionSubsystem.initializePose(
                      new Pose3d(robotContainer.swerveSubsystem.getPose()));
                }));
  }
}
