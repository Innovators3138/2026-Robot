package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem extends SubsystemBase {

  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.137, -0.19, 0.348361),
          new Rotation3d(
              0.0, 0.0, Units.degreesToRadians(-90))); // Adjust these values based on your mounting
  private static final Transform3d ROBOT_TO_SWERVE_CAM =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.18, 0.307, 0.199),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(45)));
  private static final Transform3d ROBOT_TO_SHOOTER_CAM =
      new Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.229, 0.243, 0.379),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(135)));
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

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    swervePoseEstimator = new PhotonPoseEstimator(VisionSubsystem.fieldLayout, ROBOT_TO_SWERVE_CAM);
    shooterPoseEstimator =
        new PhotonPoseEstimator(VisionSubsystem.fieldLayout, ROBOT_TO_SHOOTER_CAM);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Subsystems/Vision/Quest Pose Set", startingPoseSet);
    Logger.recordOutput("Subsystem/Vision/QuestIsConnected", questNav.isConnected());
    updateQuestNav();
    updatePose(swerveCamera, "Subsystems/Vision/SwerveCamEstimatedPose", swervePoseEstimator);
    updatePose(
        shooterCamera, "Subsystems / Vision / ShooterCamEstimatedPose", shooterPoseEstimator);
  }

  private void updatePose(PhotonCamera camera, String key, PhotonPoseEstimator poseEstimator) {
    var resultsList = camera.getAllUnreadResults();
    for (var change : resultsList) {
      if (change.hasTargets()) {
        var bestTarget = change.getBestTarget();

        Logger.recordOutput("Subsystems/Vision/BestTarget", bestTarget.getFiducialId());

        var tagCount = change.getTargets().size();
        var ambiguity = bestTarget.getPoseAmbiguity();
        var distance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
        Logger.recordOutput("Subsystems/Vision/TagCount", tagCount);
        Logger.recordOutput("Subsystems/Vision/Ambiguity", ambiguity);
        Logger.recordOutput("Subsystems/Vision/Distance", distance);
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
                Logger.recordOutput(key, estimatedPose);

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
        Logger.recordOutput("Subsystems/Vision/QuestEstimatedPose", quest2DPose);
        if (startingPoseSet == true) {
          // swerveSubsystem.addVisionMeasurement(quest2DPose, timestamp, QUESTNAV_STD_DEVS);
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
