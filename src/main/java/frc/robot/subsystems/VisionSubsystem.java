package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

class VisionSubsystem extends SubsystemBase {
  public final PhotonPoseEstimator poseEstimator;
  private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private final SwerveSubsystem swerveSubsystem;
  public List<PhotonPipelineResult> resultsList = new ArrayList<>();
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public Optional<EstimatedRobotPose> estimatedRobotPose;
  private final StructPublisher<Pose2d> estimatedPosePublisher;

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    poseEstimator =
        new PhotonPoseEstimator(
            VisionSubsystem.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Transform3d.kZero);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    estimatedPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Subsystems/Swerve/EstimatedPose", Pose2d.struct)
            .publish();
  }

  @Override
  public void periodic() {

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : resultsList) {
      visionEst = poseEstimator.update(change);
    }
    estimatedRobotPose = visionEst;
    visionEst.ifPresent(
        pose -> {
          var Hose2d = pose.estimatedPose.toPose2d();
          estimatedPosePublisher.set(Hose2d);
        });
  }
}
