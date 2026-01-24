package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
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
