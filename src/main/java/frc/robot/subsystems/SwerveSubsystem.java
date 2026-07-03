package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  public static LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(1.5);
  public static LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.5);
  public static AngularVelocity MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75);
  private boolean autoAim = false;
  private final SwerveDrive swerveDrive;

  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.DriveConstants.ROTATION_SLEW_RATE);

  private double smoothDriveInput(DoubleSupplier input, double deadband, SlewRateLimiter limiter) {
    var inputValue = MathUtil.applyDeadband(-input.getAsDouble(), deadband);

    return limiter.calculate(inputValue);
  }

  public Double targetOffset = 0.0;

  public SwerveSubsystem() {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo");
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(
                  MAX_DRIVE_SPEED.in(MetersPerSecond), Constants.FieldConstants.getInitialPose());

      swerveDrive.setMaximumAllowableSpeeds(
          MAX_DRIVE_SPEED.in(MetersPerSecond), MAX_ROTATION_SPEED.in(RadiansPerSecond));

    } catch (IOException ex) {
      throw new RuntimeException(ex);
    }

    setUpAutoPlanner();
  }

  public void setUpAutoPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings(); // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a
          // starting
          // pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->
              driveRobotRelative(
                  speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
          // Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // controller for holonomic drive trains
              new PIDConstants(3, 0.0, 0), // Translation PID constants
              new PIDConstants(4, 0.0, 0) // Rotation PID constants
              ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {

    swerveDrive.drive(speeds);
  }

  public Command resetSimOdometry() {
    return runOnce(
        () -> {
          getSimulatedPose()
              .ifPresent(
                  pose -> {
                    resetOdometry(pose);
                  });
        });
  }

  public Command driveFieldOriented(CommandXboxController driverController) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(
                swerveDrive,
                () -> {
                  return driverController.getLeftY() * -1;
                  // return yLimiter.calculate(driverController.getLeftY() * -1);
                },
                () -> {
                  return driverController.getLeftX() * -1;
                  // return xLimiter.calculate(driverController.getLeftX() * -1);
                })
            /* () -> driverController.getLeftY() * -0.5,
            () -> driverController.getLeftX() * -0.5)
             uncomment this for testing in the shop */
            .withControllerRotationAxis(
                () -> {
                  // return driverController.getRightX() * -1;
                  return rotationLimiter.calculate(driverController.getRightX() * -1);
                })
            .deadband(0.05)
            .cubeTranslationControllerAxis(true)
            .cubeRotationControllerAxis(true)
            .aimWhile(() -> autoAim)
            .allianceRelativeControl(true);

    return run(
        () -> {
          var target = Constants.FieldConstants.getHub();
          if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            target = target.transformBy(new Transform2d(0, targetOffset, Rotation2d.kZero));
          } else {
            target = target.transformBy(new Transform2d(0, -targetOffset, Rotation2d.kZero));
          }

          inputStream.aim(target);
          swerveDrive.driveFieldOriented(inputStream.get());
        });
  }

  public Command driveRobotOriented(
      CommandXboxController driverController, CommandXboxController operatorController) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(
                swerveDrive,
                () -> driverController.getLeftY() * -1,
                () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
            .deadband(0.1)
            .aimWhile(() -> operatorController.getLeftTriggerAxis() > 0.5 || autoAim);

    return run(
        () -> {
          var target = Constants.FieldConstants.getHub();

          inputStream.aim(target);

          swerveDrive.drive(inputStream.get());
        });
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0,
        5.0,
        3.0);
  }

  public Command autoAimCommand() {
    return Commands.startEnd(() -> autoAim = true, () -> autoAim = false)
        .finallyDo(() -> autoAim = false);
  }

  @Override
  public void periodic() {
    // Get the latest pose data frames from the Quest

    Logger.recordOutput("Subsystems/Swerve/EstimatedPose", swerveDrive.getPose());
  }

  public void addVisionMeasurement(
      Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive
        .getSimulationDriveTrainPose()
        .ifPresent(
            pose -> {
              Logger.recordOutput("Subsystems/Swerve/SimulatedPose", pose);
            });

    ;
  }

  public Optional<Pose2d> getSimulatedPose() {
    return swerveDrive.getSimulationDriveTrainPose();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d robotPose) {
    swerveDrive.resetOdometry(robotPose);
  }

  public Command resetOdometry() {

    return runOnce(() -> swerveDrive.resetOdometry(FieldConstants.KZERO));
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command drive(Translation2d translation, double rotation, boolean fieldRelitive) {
    return run(() -> swerveDrive.drive(translation, rotation, false, false));
  }

  public Command drivetoPose(Pose2d pose) {
    PathConstraints constraints =
        new PathConstraints(
            MAX_DRIVE_SPEED.in(MetersPerSecond),
            MAX_ACCELERATION.in(MetersPerSecondPerSecond),
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));
    return drivetoPose(pose, constraints);
  }

  public Command drivetoPose(Pose2d pose, PathConstraints pathConstraints) {

    return AutoBuilder.pathfindToPose(pose, pathConstraints, MetersPerSecond.of(0));
  }

  public double calculateHubAngle() {
    var hubPose = Constants.FieldConstants.getHub();
    var robotPose = getPose();
    var hubRelativeRelativeRobot = hubPose.relativeTo(robotPose);
    var dx = hubRelativeRelativeRobot.getX();
    var dy = hubRelativeRelativeRobot.getY();

    var relativeYaw = new Rotation2d(dx, dy);
    var angle = relativeYaw.getDegrees();
    Logger.recordOutput("Subsystems/Swerve/Hub angle", angle);
    return angle;
  }
}
