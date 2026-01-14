package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive m_SwerveDrive;

  private final StructPublisher<Pose2d> m_posePublisher =
      NetworkTableInstance.getDefault()
          .getTable("SwerveSubsystem")
          .getStructTopic("swerve/pose", Pose2d.struct)
          .publish();

  public SwerveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/neo);");
    try {
      m_SwerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(1, new Pose2d(1, 4, Rotation2d.kZero));
    } catch (IOException ex) {
      throw new RuntimeException(ex);
    }
  }

  public void driveRobotOrientated(ChassisSpeeds speeds) {
    m_SwerveDrive.drive(speeds);
  }

  public void driveFieldOrientated(ChassisSpeeds speeds) {
    m_SwerveDrive.driveFieldOriented(speeds);
  }

  public SwerveInputStream getInputStream(CommandXboxController controller) {
    return SwerveInputStream.of(
        m_SwerveDrive, () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1);
  }

  @Override
  public void periodic() {
    m_SwerveDrive.updateOdometry();
    m_posePublisher.set(m_SwerveDrive.getPose());
  }
}
