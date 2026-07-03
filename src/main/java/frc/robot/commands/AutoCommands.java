package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoCommands {
  public static Pose2d BLUE_TARGET = new Pose2d(2.5, 4.3, Rotation2d.kZero);

  public static Pose2d BLUE_COLLECTION_ZONE = new Pose2d(6.71, 4.3, Rotation2d.kCCW_90deg);
  public static SendableChooser<String> startingChooser = new SendableChooser<>();
  public static SendableChooser<String> intakeChooser = new SendableChooser<>();
  public double axisMultiplier;

  public static Pose2d getLaunchPose() {
    var basePosition = Constants.FieldConstants.forCurrentAllience(BLUE_TARGET);

    return basePosition;
  }

  public static Pose2d getMiddleStartingPose() {
    var basePosition =
        Constants.FieldConstants.forCurrentAllience(
            Constants.FieldConstants.BLUE_MIDDLE_STARTING_POSE);
    return basePosition;
  }

  public static Pose2d getCollectionPose() {
    var collectionZone = Constants.FieldConstants.forCurrentAllience(BLUE_COLLECTION_ZONE);
    return collectionZone;
  }

  public static Pose2d getStartingPosition() {
    var selectedStart = startingChooser.getSelected();
    if (selectedStart.equals("Right Start")) {

      return Constants.FieldConstants.getRightStart();
    } else if (selectedStart.equals("Middle Start")) {

      return getMiddleStartingPose();
    } else if (selectedStart.equals("Left Start")) {

      return Constants.FieldConstants.getLeftStart();
    } else if (selectedStart.equals("Left Trench Start")) {
      return Constants.FieldConstants.getLeftTrenchStart();
    } else {
      return Constants.FieldConstants.getRightTrenchStart();
    }
  }

  private static Command getShootingCommand(RobotContainer robotContainer) {
    var selectedStart = startingChooser.getSelected();
    if (selectedStart.equals("Right Start") || selectedStart.equals("Right Trench Start")) {

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
    PathConstraints slowConstraints =
        new PathConstraints(0.50, 0.50, Units.degreesToRadians(360), Units.degreesToRadians(360));
    PathConstraints fastConstraints =
        new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    if (selectedIntake.equals("Depot Intake")) {
      return robotContainer
          .intakeSubsystem
          .openHopper()
          .andThen(robotContainer.intakeSubsystem.setAngularVelocity(RPM.of(2500)))
          .raceWith(
              robotContainer
                  .swerveSubsystem
                  .drivetoPose(Constants.FieldConstants.getDepot(), slowConstraints)
                  .andThen(Commands.waitSeconds(1)));
    } else if (selectedIntake.equals("Left Intake")) {
      return robotContainer
          .intakeSubsystem
          .openHopper()
          .andThen(Commands.waitSeconds(1))
          .andThen(robotContainer.intakeSubsystem.setAngularVelocity(RPM.of(2500)))
          .raceWith(
              AutoBuilder.pathfindThenFollowPath(
                  PathPlannerPath.fromPathFile("Left Intake"), fastConstraints));
    } else if (selectedIntake.equals("Right Intake")) {

      return robotContainer
          .intakeSubsystem
          .openHopper()
          .andThen(Commands.waitSeconds(1))
          .andThen(robotContainer.intakeSubsystem.setAngularVelocity(RPM.of(2500)))
          .raceWith(
              AutoBuilder.pathfindThenFollowPath(
                  PathPlannerPath.fromPathFile("Right Intake"), fastConstraints));
    } else {
      return Commands.none();
    }
  }

  private static Command initializeStartingPose(RobotContainer robotContainer) {

    return Commands.waitUntil(() -> robotContainer.visionSubsystem.questNav.isConnected())
        .withTimeout(1)
        .andThen(
            Commands.runOnce(
                () -> {
                  // robotContainer.visionSubsystem.resetQuestPose(new
                  // Pose3d(getStartingPosition()));
                  robotContainer.swerveSubsystem.resetOdometry(getStartingPosition());
                }));
  }

  public static Command createAuto(RobotContainer robotContainer)
      throws FileVersionException, IOException, ParseException {
    Command driveToShootingCommand = getShootingCommand(robotContainer);
    Command driveToLoadingCommand = getLoadingCommand(robotContainer);

    return driveToLoadingCommand
        .andThen(driveToShootingCommand)
        .andThen(
            FireCommand.targetLock(robotContainer.shooterSubsystem, robotContainer.swerveSubsystem)
                .alongWith(robotContainer.swerveSubsystem.autoAimCommand())
                .withTimeout(1.5))
        .andThen(
            FireCommand.fire(
                    robotContainer.feederSubsystem,
                    robotContainer.hotdogSubsystem,
                    robotContainer.shooterSubsystem)
                .withTimeout(5));
  }
}
