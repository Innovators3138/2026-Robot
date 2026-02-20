package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  private final SwerveSubsystem swerveSubsystem;


  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void periodic() {

     var visionEst =  camera.getAllUnreadResults();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs();

                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

  }


}
