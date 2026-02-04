package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final SolidColor greenColor = new SolidColor(0, 50);
  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final FeederSubsystem feederSubsystem;


  public LEDSubsystem(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.candle = new CANdle(36);
  }


  public Command setToGreen() {

    return runOnce(
        () -> {
          var feederCheck = feederSubsystem.feederGetAngularVelocity().minus(RPM.of(2400)).abs(RPM);
           if (feederCheck <= 20)  {
              greenColor.withColor(green);
              candle.setControl(greenColor);}
            } );

  }
}
