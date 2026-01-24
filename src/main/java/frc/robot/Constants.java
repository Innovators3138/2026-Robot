package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
  public static Pose2d BLUE_HUB = new Pose2d(4.341067, 4, Rotation2d.kZero);
  public static Pose2d RED_HUB = new Pose2d(11.796345, 4.37278, Rotation2d.kZero);

  public static Pose2d getHub() {
    var basePosition = Constants.BLUE_HUB;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      basePosition = Constants.RED_HUB;
    }
    return basePosition;
  }

  public static final class AutoConstants {
    public static final LinearVelocity AUTO_DRIVE_VELOCITY = MetersPerSecond.of(2.0);
    public static final LinearAcceleration AUTO_DRIVE_ACCELERATION =
        MetersPerSecondPerSecond.of(2.0);
  }
}
