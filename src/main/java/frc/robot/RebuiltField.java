package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RebuiltField {
  public static Pose2d BLUE_HUB = new Pose2d(4.341067, 4, Rotation2d.kZero);
  public static Pose2d RED_HUB = new Pose2d(11.796345, 4.37278, Rotation2d.kZero);

  public static Pose2d getHub() {
    var target = RebuiltField.BLUE_HUB;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      target = RebuiltField.RED_HUB;
    }
    return target;
  }
}
