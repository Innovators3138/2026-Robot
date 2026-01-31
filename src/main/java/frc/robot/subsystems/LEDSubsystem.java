package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final SolidColor Greencolor = new SolidColor(0, 50);
  private final RGBWColor Green = new RGBWColor(0, 255, 0, 0);

  public LEDSubsystem() {
    this.candle = new CANdle(36);
  }

  public Command setToGreen() {
    return runOnce(
        () -> {
          Greencolor.withColor(Green);
          candle.setControl(Greencolor);
        });
  }
}
