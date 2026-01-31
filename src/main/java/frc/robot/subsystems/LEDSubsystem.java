package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final SolidColor Redcolor = new SolidColor(0, 50);
  private final RGBWColor Red = new RGBWColor(255, 0, 0, 0);

  public LEDSubsystem() {
    this.candle = new CANdle(36);
  }

  public Command setToRed() {
    return runOnce(
        () -> {
          Redcolor.withColor(Red);
          candle.setControl(Redcolor);
        });
  }
}
