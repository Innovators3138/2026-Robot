package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;
 // private final SolidColor greenColor = new SolidColor(0, 50);
 // private final SolidColor redColor = new SolidColor(0, 50);
  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor red = new RGBWColor(255, 0, 0, 0);
  private final ShooterSubsystem shooterSubsystem;
  private final IntegerPublisher candlePublisher =
      NetworkTableInstance.getDefault().getIntegerTopic("Subsystems/Ledsubsystem/candle").publish();

  public LEDSubsystem(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.candle = new CANdle(21);
  }
  private double getGetSetpointAccuracyPercentage() {
    var realSpeed = shooterSubsystem.getRealAngularVelocity().in(RPM);
    var setpoint = shooterSubsystem.getAngularVelocitySetpoint().map(s -> s.in(RPM)).orElse(0.0);
    if (setpoint == 0){
      return 0;
    }
    var percentage = realSpeed/setpoint;

    return percentage;

    }
@Override
  public void periodic() {
    var percentage = getGetSetpointAccuracyPercentage();
    var Led = LEDConstants.LED_NUMBER;
    var level = (int)(percentage * Led);
     RGBWColor color;
    if (percentage > 1) {
       color = red;
    }else{
       color = green;
    }
    candle.setControl(new SolidColor(8, level + 8).withColor(color));


  }
}





