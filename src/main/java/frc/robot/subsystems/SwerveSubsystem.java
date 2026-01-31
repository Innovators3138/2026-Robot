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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(0.307, -0.254000, 0.322762),
          new Rotation3d(0.0, 0.0, 0.0)); // Adjust these values based on your mounting
  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );
  public static LinearVelocity MaxDriveSpeed = MetersPerSecond.of(5);
  public static AngularVelocity MaxRotationSpeed = RotationsPerSecond.of(3);

  private final SwerveDrive swerveDrive;
  private final StructPublisher<Pose2d> estimatedPosePublisher;
  private final StructPublisher<Pose2d> simulatedPosePublisher;
  private final QuestNav questNav = new QuestNav();

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
              .createSwerveDrive(
                  MaxDriveSpeed.in(MetersPerSecond), Constants.FieldConstants.getInitialPose());

      swerveDrive.setMaximumAllowableSpeeds(
          MaxDriveSpeed.in(MetersPerSecond), MaxRotationSpeed.in(RadiansPerSecond));
    } catch (IOException ex) {
      throw new RuntimeException(ex);
    }
    setupQuestNav(new QuestNav());
    setUpAutoPlanner();
  }

  public void setupQuestNav(QuestNav questNav) {

    Pose3d robotPose3d = new Pose3d(getPose());
    Pose3d questPose = robotPose3d.transformBy(ROBOT_TO_QUEST);
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
          var target = Constants.FieldConstants.getHub();

          inputStream.aim(target);
          swerveDrive.driveFieldOriented(inputStream.get());
        });
  }

  @Override
  public void periodic() {
    // Get the latest pose data frames from the Quest

    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        swerveDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        swerveDrive.updateOdometry();
      }
    }
    estimatedPosePublisher.set(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive
        .getSimulationDriveTrainPose()
        .ifPresent(
            pose -> {
              simulatedPosePublisher.set(pose);
              swerveDrive.addVisionMeasurement(
                  pose, Timer.getFPGATimestamp() - 0.02, QUESTNAV_STD_DEVS);
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
