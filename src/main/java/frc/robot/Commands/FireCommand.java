package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class FireCommand extends Command {
  static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  static {
    distanceToRPM.put(2.5, 1350.0);
    distanceToRPM.put(2.97, 1600.0);
    distanceToRPM.put(3.48, 1750.0);
    distanceToRPM.put(4.0, 1900.0);
    distanceToRPM.put(4.57, 2050.0);
    distanceToRPM.put(4.95, 2150.0);
    distanceToRPM.put(5.54, 2300.0);
    distanceToRPM.put(6.98, 2650.0);
  }

  public static Command targetLock(
      ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    return shooterSubsystem.setAngularVelocity(
        () -> {
          var translation = swerveSubsystem.getPose().getTranslation();
          var target = Constants.FieldConstants.getHub();
          var distance = target.getTranslation().getDistance(translation);
          var distanceInMeters = Meter.of(distance).plus(ShooterSubsystem.SHOOTER_OFFSET_X);
          var shooterSpeed = distanceToRPM.get(distanceInMeters.in(Meter));
          return RPM.of(shooterSpeed);
        });
  }

  public static Command fire(FeederSubsystem feedersubsystem, HotdogSubsystem hotdogsubsystem) {
    return feedersubsystem
        .setFeederAngularVelocity(RPM.of(2400))
        .alongWith(hotdogsubsystem.setHotdogAngularVelocity(RPM.of(180)));
  }
}
