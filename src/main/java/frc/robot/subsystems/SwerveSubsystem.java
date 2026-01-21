package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RebuiltField;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  public static LinearVelocity MaxDriveSpeed = MetersPerSecond.of(5);
  public static AngularVelocity MaxRotationSpeed = RotationsPerSecond.of(3);
  public static Pose2d InitialPose = new Pose2d(16, 7, Rotation2d.kZero);

  private final SwerveDrive swerveDrive;
  private final StructPublisher<Pose2d> posePublisher;

  public SwerveSubsystem() {
    posePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Subsystems/Swerve/Pose", Pose2d.struct)
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
          var target = RebuiltField.BLUE_HUB;
          if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            target = RebuiltField.RED_HUB;
          }

          inputStream.aim(target);
          swerveDrive.driveFieldOriented(inputStream.get());
        });
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    posePublisher.set(swerveDrive.getPose());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }
}
