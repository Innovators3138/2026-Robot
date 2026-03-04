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

public class TestVisionSubsystem extends SubsystemBase {
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


  public TestVisionSubsystem() {

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

    updatePose(swerveCamera, swerveEstimatedPosePublisher, swervePoseEstimator);
    updatePose(shooterCamera, shooterEstimatedPosePublisher, shooterPoseEstimator);
  }

  private void updatePose(
      PhotonCamera camera, StructPublisher<Pose2d> publisher, PhotonPoseEstimator poseEstimator) {
    var resultsList = camera.getAllUnreadResults();
    for (var change : resultsList) {
      var visionEst = poseEstimator.update(change);
      boolean hasTarget = change.hasTargets();
      if (hasTarget) {
      var ambiguity = change.getBestTarget().getPoseAmbiguity();

      if (ambiguity < 0.2) {
        visionEst.ifPresent(
            pose -> {
              var pose2d = pose.estimatedPose.toPose2d();
              publisher.set(pose2d);

            });
          }
      }
    }
  }


}
