package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

public class SwerveTestSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for
    // this kind of control.
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
}
