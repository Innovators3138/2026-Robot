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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem extends SubsystemBase {
  private final IntegerPublisher bestTargetPublisher =
      NetworkTableInstance.getDefault().getIntegerTopic("Subsystems/Vision/BestTarget").publish();
  private final IntegerPublisher tagCountPublisher =
      NetworkTableInstance.getDefault().getIntegerTopic("Subsystems/Vision/TagCount").publish();
  private final DoublePublisher ambiguityPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Vision/Ambiguity").publish();
  private final DoublePublisher distancePublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("Subsystems/Vision/Distance").publish();
  private final BooleanPublisher poseIsSetPublisher =
      NetworkTableInstance.getDefault()
          .getBooleanTopic("Subsystems/Vision/Quest Pose Set")
          .publish();

  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.1398778, 0.195199, 0.348361),
          new Rotation3d(0.0, 0.0, 4.71238898)); // Adjust these values based on your mounting
  private static final Transform3d ROBOT_TO_SWERVE_CAM =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.18, 0.307, 0.199),
          new Rotation3d(0.0, 0.0, 0.78539816));
  private static final Transform3d ROBOT_TO_SHOOTER_CAM =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.229, 0.243, 0.379),
          new Rotation3d(0.0, 0.0, 2.35619449));
  private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 3.5 degrees rotational
          );
  private static final Matrix<N3, N1> CAMERA_STD_DEVS =
      VecBuilder.fill(
          0.1, // Trust down to 2cm in X direction
          0.1, // Trust down to 2cm in Y direction
          0.1 // Trust down to 2 degrees rotational
          );
  public final PhotonPoseEstimator swervePoseEstimator;
  public final PhotonPoseEstimator shooterPoseEstimator;
  private final PhotonCamera swerveCamera = new PhotonCamera("Arducam-2");
  private final PhotonCamera shooterCamera = new PhotonCamera("Arducam-1");
  private final QuestNav questNav = new QuestNav();
  private final SwerveSubsystem swerveSubsystem;
  private boolean startingPoseSet = false;
  public BooleanPublisher connectionPublisher =
      NetworkTableInstance.getDefault()
          .getBooleanTopic("Subsystem/Vision/QuestIsConnected")
          .publish();

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
    swervePoseEstimator = new PhotonPoseEstimator(VisionSubsystem.fieldLayout, ROBOT_TO_SWERVE_CAM);
    shooterPoseEstimator =
        new PhotonPoseEstimator(VisionSubsystem.fieldLayout, ROBOT_TO_SHOOTER_CAM);
  }

  @Override
  public void periodic() {
    poseIsSetPublisher.set(startingPoseSet);
    connectionPublisher.set(questNav.isConnected());
    updateQuestNav();
    updatePose(swerveCamera, swerveEstimatedPosePublisher, swervePoseEstimator);
    updatePose(shooterCamera, shooterEstimatedPosePublisher, shooterPoseEstimator);
  }

  private void updatePose(
      PhotonCamera camera, StructPublisher<Pose2d> publisher, PhotonPoseEstimator poseEstimator) {
    var resultsList = camera.getAllUnreadResults();
    for (var change : resultsList) {
      if (change.hasTargets()) {
        var bestTarget = change.getBestTarget();

        bestTargetPublisher.set(bestTarget.getFiducialId());

        var tagCount = change.getTargets().size();
        var ambiguity = bestTarget.getPoseAmbiguity();
        var distance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
        tagCountPublisher.set(tagCount);
        ambiguityPublisher.set(ambiguity);
        distancePublisher.set(distance);
        if (ambiguity >= 0 && ambiguity < 0.2 && (tagCount >= 2 || distance < 3.0)) {
          var visionEst = poseEstimator.estimateCoprocMultiTagPose(change);

          if (visionEst.isEmpty()) {
            visionEst = poseEstimator.estimateLowestAmbiguityPose(change);
          }

          visionEst.ifPresent(
              estimate -> {
                var estimatedPose = estimate.estimatedPose.toPose2d();

                var changeInDistance =
                    swerveSubsystem
                        .getPose()
                        .getTranslation()
                        .getDistance(estimatedPose.getTranslation());
                publisher.set(estimatedPose);

                if ((startingPoseSet == true && changeInDistance < 1 || startingPoseSet == false)) {
                  swerveSubsystem.addVisionMeasurement(
                      estimatedPose, estimate.timestampSeconds, CAMERA_STD_DEVS);
                }
              });
        }
      }
    }
  }

  // }

  private void updateQuestNav() {
    // trust me
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
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
        questEstimatedPosePublisher.set(quest2DPose);
        if (startingPoseSet == true) {
          swerveSubsystem.addVisionMeasurement(quest2DPose, timestamp, QUESTNAV_STD_DEVS);
        }
      }
    }
  }

  public void initializePose(Pose3d initialPose) {

    if (questNav.isConnected()) {
      questNav.setPose(initialPose);
      startingPoseSet = true;
    }
  }
}
