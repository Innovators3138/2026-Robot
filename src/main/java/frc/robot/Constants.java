package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {

  public static final class FieldConstants {
    public static Pose2d BLUE_HUB = new Pose2d(4.625594, 4.034663, Rotation2d.kZero);

    public static Pose2d BLUE_STARTING_POSITION = new Pose2d(1, 1, Rotation2d.kZero);
    public static Pose2d KZERO = new Pose2d(1, 1.0, Rotation2d.kZero);
    public static Pose2d BLUE_DEPOT = new Pose2d(0.95, 5.833, Rotation2d.k180deg);
    public static Pose2d BLUE_SHOOT_RIGHT = new Pose2d(2.004, 4.956, Rotation2d.fromDegrees(-28));
    public static Pose2d BLUE_SHOOT_LEFT = new Pose2d(2.052, 2.688, Rotation2d.fromDegrees(32.07));
    public static Pose2d BLUE_SHOOT_MIDDLE = new Pose2d(2.5, 4.037, Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_START_RIGHT = new Pose2d(3.6, 1.97, Rotation2d.kZero);

    public static Pose2d BLUE_START_LEFT = new Pose2d(3.6, 6.115, Rotation2d.k180deg);

    public static Pose2d getRightShoot() {
      var leftShootPosition = forCurrentAllience(BLUE_SHOOT_LEFT);

      return leftShootPosition;
    }

    public static Pose2d getRightStart() {
      var rightStartPosition = forCurrentAllience(BLUE_START_RIGHT);
      return rightStartPosition;
    }

    public static Pose2d getLeftStart() {
      var leftStartPosition = forCurrentAllience(BLUE_START_LEFT);
      return leftStartPosition;
    }

    public static Pose2d getMiddleShoot() {
      var middleShootPosition = forCurrentAllience(BLUE_SHOOT_MIDDLE);
      return middleShootPosition;
    }

    public static Pose2d getLeftShoot() {
      var rightShootPosition = forCurrentAllience(BLUE_SHOOT_RIGHT);
      return rightShootPosition;
    }

    // get left shoot doesnt work for whatever reason
    public static Pose2d getHub() {
      var basePosition = forCurrentAllience(BLUE_HUB);

      return basePosition;
    }

    public static Pose2d getDepot() {
      var depotPosition = forCurrentAllience(BLUE_DEPOT);

      return depotPosition;
    }

    public static Pose2d getInitialPose() {
      var basePosition = forCurrentAllience(BLUE_STARTING_POSITION);
      return basePosition;
    }

    public static Pose2d forCurrentAllience(Pose2d bluepose) {
      if (!isRedAlliance()) {
        return bluepose;
      }
      return new Pose2d(
          16.540988 - bluepose.getX(),
          8.069326 - bluepose.getY(),
          bluepose.getRotation().rotateBy(Rotation2d.kPi));
    }

    private static boolean isRedAlliance() {
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
  }

  public static final class AutoConstants {
    public static final LinearVelocity AUTO_DRIVE_VELOCITY = MetersPerSecond.of(2.0);
    public static final LinearAcceleration AUTO_DRIVE_ACCELERATION =
        MetersPerSecondPerSecond.of(2.0);
  }

  public static final class IntakeConstants {

    public static final double MINIMUM_SIMULATED_INTAKE_DISTANCE = 1.0;
    public static final int SIMULATED_CAPACITY = 30;
  }

  public static final class LEDConstants {
    public static final int LED_SPEED_NUMBER = 20;
    public static final int LED_ANGLE_NUMBER = 21;
    // angle number must be an odd number of leds so there is a middle one
    public static final double LED_DEADZONE = 8;
    public static final int LED_INCREMENT = 9;
    public static final int LED_MULTIPLIER = 2;
    public static final int TOTAL_LEDS = 48;
  }

  public static final class DriveConstants {
    public static final double TRANSLATION_SLEW_RATE = 12.0;
    public static final double ROTATION_SLEW_RATE = 6.0;
  }
}
