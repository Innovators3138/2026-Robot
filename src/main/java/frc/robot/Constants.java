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

    public static Pose2d getHub() {
      var basePosition = Constants.FieldConstants.BLUE_HUB;
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        basePosition = Constants.FieldConstants.RED_HUB;
      }
      return basePosition;
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
}
