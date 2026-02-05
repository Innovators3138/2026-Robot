package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final SolidColor greenColor = new SolidColor(0, 50);
  private final SolidColor redColor = new SolidColor(0, 50);
  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor red = new RGBWColor(255, 0, 0, 0);
  private final ShooterSubsystem shooterSubsystem;
  private final IntegerPublisher candlePublisher =
      NetworkTableInstance.getDefault().getIntegerTopic("Subsystems/Ledsubsystem/candle").publish();

  public LEDSubsystem(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.candle = new CANdle(36);
  }

  public Command setToGreen() {

    return runOnce(
        () -> {
          var shooterCheck =
              shooterSubsystem
                  .getRealAngularVelocity()
                  .minus(shooterSubsystem.getAngularVelocity().orElse(RPM.of(0)))
                  .abs(RPM);
          if (shooterCheck <= 20
              && shooterSubsystem.getAngularVelocity().orElse(RPM.of(0)).gt(RPM.of(0))) {
            greenColor.withColor(green);
            candle.setControl(greenColor);
            candlePublisher.set(1);

          } else {
            redColor.withColor(red);
            candle.setControl(redColor);
            candlePublisher.set(0);
          }
        });
  }
}
