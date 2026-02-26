package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {
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
  public final PhotonPoseEstimator swervePoseEstimator;
  public final PhotonPoseEstimator shooterPoseEstimator;
  private final PhotonCamera swerveCamera = new PhotonCamera("Arducam-1");
  private final PhotonCamera shooterCamera = new PhotonCamera("Arducam-2");
  private final QuestNav questNav = new QuestNav();
  private final SwerveSubsystem swerveSubsystem;

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private final StructPublisher<Pose2d> swerveEstimatedPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Subsystems/Vision/SwerveCamEstimatedPose", Pose2d.struct)
          .publish();

  private final StructPublisher<Pose2d> shooterEstimatedPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Subsystems/Vision/ShooterCamEstimatedPose", Pose2d.struct)
          .publish();
  private final StructPublisher<Pose2d> questEstimatedPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Subsystems/Vision/QuestEstimatedPose", Pose2d.struct)
          .publish();

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    swervePoseEstimator =
        new PhotonPoseEstimator(
            VisionSubsystem.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Transform3d.kZero);
    swervePoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    shooterPoseEstimator =
        new PhotonPoseEstimator(
            VisionSubsystem.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Transform3d.kZero);
    shooterPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    updateQuestNav();
    updatePose(swerveCamera, swerveEstimatedPosePublisher, swervePoseEstimator);
    updatePose(shooterCamera, shooterEstimatedPosePublisher, shooterPoseEstimator);
  }

  private void updatePose(
      PhotonCamera camera, StructPublisher<Pose2d> publisher, PhotonPoseEstimator poseEstimator) {
    var resultsList = camera.getAllUnreadResults();
    for (var change : resultsList) {
      var visionEst = poseEstimator.update(change);
      var ambiguity = change.getBestTarget().getPoseAmbiguity();
      if (ambiguity < 0.2) {
        visionEst.ifPresent(
            pose -> {
              var pose2d = pose.estimatedPose.toPose2d();
              publisher.set(pose2d);
              swerveSubsystem.addVisionMeasurement(pose2d, pose.timestampSeconds);
            });
      }
    }
  }

  private void updateQuestNav() {
    // trust me
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
        var quest2DPose = robotPose.toPose2d();
        swerveSubsystem.addVisionMeasurement(quest2DPose, timestamp);
        questEstimatedPosePublisher.set(quest2DPose);
      }
    }
  }
}
