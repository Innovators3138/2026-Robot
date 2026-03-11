package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {

  public static final class FieldConstants {
    public static Pose2d BLUE_HUB = new Pose2d(4.341067, 4, Rotation2d.kZero);
    public static Pose2d RED_HUB = new Pose2d(11.796345, 4.37278, Rotation2d.kZero);
    public static Pose2d BLUE_STARTING_POSITION = new Pose2d(1, 1, Rotation2d.kZero);
    public static Pose2d RED_STARTING_POSITION = new Pose2d(16, 8, Rotation2d.k180deg);
    public static Pose2d KZERO = new Pose2d(1, 1.0, Rotation2d.kZero);
    public static Pose2d RED_CLIMB_RIGHT = new Pose2d(15.71, 5.261, Rotation2d.kZero);
    public static Pose2d RED_CLIMB_LEFT = new Pose2d(15.71, 3.396, Rotation2d.kZero);
    public static Pose2d RED_CLIMB_MIDDLE = new Pose2d(15.1, 4.341, Rotation2d.kZero);
    public static Pose2d BLUE_CLIMB_RIGHT = new Pose2d(1.057, 5.05, Rotation2d.kZero);
    public static Pose2d BLUE_CLIMB_MIDDLE = new Pose2d(1.439, 3.825, Rotation2d.kZero);
    public static Pose2d BLUE_CLIMB_LEFT = new Pose2d(1.217, 3.108, Rotation2d.kZero);
    public static Pose2d RED_DEPOT = new Pose2d(15.606, 1.977, Rotation2d.k180deg);
    public static Pose2d BLUE_DEPOT = new Pose2d(0.34, 5.833, Rotation2d.k180deg);
    public static Pose2d BLUE_SHOOT_LEFT = new Pose2d(2.004, 4.956, Rotation2d.fromDegrees(-28));
    public static Pose2d BLUE_SHOOT_RIGHT = new Pose2d(2.228, 2.688, Rotation2d.fromDegrees(32.07));
    public static Pose2d BLUE_SHOOT_MIDDLE = new Pose2d(2.512, 4.095, Rotation2d.fromDegrees(0));

    public static Pose2d RED_SHOOT_LEFT =
        new Pose2d(14.610701, 3.261313, Rotation2d.fromDegrees(158.53));
    public static Pose2d RED_SHOOT_MIDDLE = new Pose2d(14.486, 4.454, Rotation2d.fromDegrees(0));
    public static Pose2d RED_SHOOT_RIGHT =
        new Pose2d(14.51, 5.347, Rotation2d.fromDegrees(-157.03));

    public static Pose2d getLeftShoot() {
      var leftShootPosition = Constants.FieldConstants.BLUE_SHOOT_LEFT;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        leftShootPosition = Constants.FieldConstants.RED_SHOOT_LEFT;
      }
      return leftShootPosition;
    }

    public static Pose2d getMiddleShoot() {
      var middleShootPosition = Constants.FieldConstants.BLUE_SHOOT_MIDDLE;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        middleShootPosition = Constants.FieldConstants.RED_SHOOT_MIDDLE;
      }
      return middleShootPosition;
    }

    public static Pose2d getRightShoot() {
      var rightShootPosition = Constants.FieldConstants.BLUE_SHOOT_RIGHT;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        rightShootPosition = Constants.FieldConstants.RED_SHOOT_RIGHT;
      }
      return rightShootPosition;
    }

    // get left shoot doesnt work for whatever reason
    public static Pose2d getHub() {
      var basePosition = Constants.FieldConstants.BLUE_HUB;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        basePosition = Constants.FieldConstants.RED_HUB;
      }
      return basePosition;
    }

    public static Pose2d getDepot() {
      var depotPosition = Constants.FieldConstants.BLUE_DEPOT;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        depotPosition = Constants.FieldConstants.RED_DEPOT;
      }
      return depotPosition;
    }

    public static Pose2d getRightClimb() {
      var leftClimb = Constants.FieldConstants.BLUE_CLIMB_RIGHT;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        leftClimb = Constants.FieldConstants.RED_CLIMB_RIGHT;
      }
      return leftClimb;
    }

    public static Pose2d getLeftClimb() {
      var rightClimb = Constants.FieldConstants.BLUE_CLIMB_LEFT;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        rightClimb = Constants.FieldConstants.RED_CLIMB_LEFT;
      }
      return rightClimb;
    }

    public static Pose2d getMiddleClimb() {
      var middleClimb = Constants.FieldConstants.BLUE_CLIMB_MIDDLE;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        middleClimb = Constants.FieldConstants.RED_CLIMB_MIDDLE;
      }
      return middleClimb;
    }

    public static Pose2d getInitialPose() {
      var basePosition = Constants.FieldConstants.BLUE_STARTING_POSITION;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        basePosition = Constants.FieldConstants.RED_STARTING_POSITION;
      }
      return basePosition;
    }
  }

  public static final class AutoConstants {
    public static final LinearVelocity AUTO_DRIVE_VELOCITY = MetersPerSecond.of(2.0);
    public static final LinearAcceleration AUTO_DRIVE_ACCELERATION =
        MetersPerSecondPerSecond.of(2.0);
  }

  public static final class ClimberConstants {
    public static final Distance MINIMUM_SAFE_HEIGHT = Inches.of(0.5);
    public static final Distance MAXIMUM_SAFE_HEIGHT = Inches.of(27.0);

    public static final Distance CLIMBER_INITIAL_HEIGHT = Meters.of(0.0);
    public static final Distance CLIMBER_MAX_HEIGHT = Meters.of(0.666);
    public static final boolean CLIMBER_MOTOR_IS_INVERTED = true;
    public static final boolean CLIMBER_ENCODER_IS_INVERTED = true;
    public static final double CLIMBER_ABSOLUTE_SENSOR_DISCONTINUITY_POINT = 0.95;
    public static final double CLIMBER_ABSOLUTE_SENSOR_OFFSET = 0.243;
    public static final double CLIMBER_P = 12;
    public static final double CLIMBER_I = 2.0;
    public static final double CLIMBER_D = 0.1;
    public static final double CLIMBER_FF = 0;
    public static final double CLIMBER_IZ = 0.1;

    public static final double CLIMBER_KG = 0.48;
    public static final double CLIMBER_KV = 2.66;
    public static final double CLIMBER_KA = 0.05;

    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
    public static final double CLIMBER_MOTOR_RAMP_RATE = 0.25;
    public static final LinearVelocity CLIMBER_MAX_VELOCITY = MetersPerSecond.of(1.0);

    public static final Distance CLIMBER_DRUM_DIAMETER = Inches.of(1.432);
    public static final Distance CLIMBER_CONVERSION_FACTOR =
        CLIMBER_DRUM_DIAMETER.times(
            Math.PI * 64.0 / 24.0 * 64.0 / 24.0); // Distance per Magnet Rotation
    public static final Distance CLIMBER_THRESHOLD = Inches.of(0.125);
    public static final SparkBase CLIMBER_MOTOR_ID = null;
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
  }
}
