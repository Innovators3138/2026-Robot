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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  public static LinearVelocity MaxDriveSpeed = MetersPerSecond.of(5);
  public static AngularVelocity MaxRotationSpeed = RotationsPerSecond.of(3);
  public static Pose2d InitialPose = new Pose2d(16, 7, Rotation2d.kZero);

  private final SwerveDrive swerveDrive;
  private final StructPublisher<Pose2d> estimatedPosePublisher;
  private final StructPublisher<Pose2d> simulatedPosePublisher;

  public SwerveSubsystem() {

    estimatedPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Subsystems/Swerve/EstimatedPose", Pose2d.struct)
            .publish();
    simulatedPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Subsystems/Swerve/SimulatedPose", Pose2d.struct)
            .publish();
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo");

    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(MaxDriveSpeed.in(MetersPerSecond), InitialPose);

      swerveDrive.setMaximumAllowableSpeeds(
          MaxDriveSpeed.in(MetersPerSecond), MaxRotationSpeed.in(RadiansPerSecond));
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
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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

  public Command driveFieldOriented(
      CommandXboxController driverController, CommandXboxController operatorController) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(
                swerveDrive,
                () -> driverController.getLeftY() * -1,
                () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
            .deadband(0.1)
            .allianceRelativeControl(true)
            .aimWhile(() -> operatorController.getLeftTriggerAxis() > 0.5);

    return run(
        () -> {
          var target = Constants.getHub();

          inputStream.aim(target);
          swerveDrive.driveFieldOriented(inputStream.get());
        });
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    estimatedPosePublisher.set(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive
        .getSimulationDriveTrainPose()
        .ifPresent(
            pose -> {
              simulatedPosePublisher.set(pose);
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
    swerveDrive.resetOdometry(InitialPose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command drivetoPose(
      Pose2d pose, LinearVelocity velocity, LinearAcceleration acceleration) {
    PathConstraints constraints =
        new PathConstraints(
            velocity.in(MetersPerSecond),
            acceleration.in(MetersPerSecondPerSecond),
            swerveDrive.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));
    return AutoBuilder.pathfindToPose(
        pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }
}
