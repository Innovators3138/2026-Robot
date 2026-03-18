package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Inch;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class FireCommand extends Command {
  static InterpolatingDoubleTreeMap inchesToRPS = new InterpolatingDoubleTreeMap();
  public RobotContainer robotContainer;
  public static final DoublePublisher distancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Command/FireCommand/DistanceInInches").publish();

  static {
    inchesToRPS.put(96.0, 52.5);
    inchesToRPS.put(26.0, 40.83);
    inchesToRPS.put(141.0, 60.833);
    inchesToRPS.put(157.0, 62.5);
    inchesToRPS.put(45.0, 42.5);
  }

  public static Command targetLock(
      ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    return shooterSubsystem.setAngularVelocity(
        () -> {
          var translation = swerveSubsystem.getPose().getTranslation();
          var target = Constants.FieldConstants.getHub();
          var distance = target.getTranslation().getDistance(translation);
          var distanceInInches = Meter.of(distance).plus(ShooterSubsystem.SHOOTER_OFFSET_X).in(Inch);
          var shooterSpeed = inchesToRPS.get(distanceInInches);
          distancePublisher.set(distanceInInches);
          return RotationsPerSecond.of(shooterSpeed);
        });
  }

  public static Command fire(
      FeederSubsystem feedersubsystem,
      HotdogSubsystem hotdogsubsystem,
      ShooterSubsystem shooterSubsystem) {

    return feedersubsystem
        .setFeederAngularVelocity(
            () -> {
              if (shooterSubsystem.getRealAngularVelocity().gte(RPM.of(500))) {

                return RPM.of(2400);
              } else {
                return RPM.of(0);
              }
            })
        .alongWith(
            hotdogsubsystem.setHotdogAngularVelocity(
                () -> {
                  if (shooterSubsystem.getRealAngularVelocity().gte(RPM.of(500))) {

                    return RPM.of(2400);
                  } else {
                    return RPM.of(0);
                  }
                }));
  }

  public static Command pass(
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      HotdogSubsystem hotdogsubsystem,
      DoubleSupplier speedMultiplier) {
    return shooterSubsystem
        .setAngularVelocity(() -> RPM.of(5000).times(speedMultiplier.getAsDouble()))
        .alongWith(
            feederSubsystem
                .setFeederAngularVelocity(
                    () -> {
                      if (shooterSubsystem.getRealAngularVelocity().gte(RPM.of(500))) {

                        return RPM.of(2400);
                      } else {
                        return RPM.of(0);
                      }
                    })
                .alongWith(
                    hotdogsubsystem.setHotdogAngularVelocity(
                        () -> {
                          if (shooterSubsystem.getRealAngularVelocity().gte(RPM.of(500))) {

                            return RPM.of(2400);
                          } else {
                            return RPM.of(0);
                          }
                        })));
  }

  public static Command unjam(FeederSubsystem feedersubsystem, HotdogSubsystem hotdogsubsystem) {
    return feedersubsystem
        .setFeederAngularVelocity(RPM.of(-2400))
        .alongWith(hotdogsubsystem.setHotdogAngularVelocity(RPM.of(-180)));
  }

  // uh, comment
}
