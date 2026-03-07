package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;

  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor red = new RGBWColor(255, 0, 0, 0);
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final IntegerPublisher flywheelLEDPublisher =
      NetworkTableInstance.getDefault()
          .getIntegerTopic("Subsystems/Ledsubsystem/FlywheelLEDs")
          .publish();

  public LEDSubsystem(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.candle = new CANdle(21);
  }

  private double getGetSetpointAccuracyPercentage() {
    var realSpeed = shooterSubsystem.getRealAngularVelocity().in(RPM);
    var setpoint = shooterSubsystem.getAngularVelocitySetpoint().map(s -> s.in(RPM)).orElse(0.0);
    if (setpoint <= 0) {
      return 0;
    }
    var percentage = realSpeed / setpoint;

    return percentage;
  }
  private void postTheColors() {
    var percentage = getGetSetpointAccuracyPercentage();
    var led = LEDConstants.LED_NUMBER;
    var level = (int) (percentage * led);
    RGBWColor color;
    if (percentage > 1) {
      color = red;
      level = 20;
    } else {
      color = green;
    }
    candle.setControl(new SolidColor(8, level + 8).withColor(color));
    flywheelLEDPublisher.set(level);}

  @Override
  public void periodic() {
   postTheColors();
  }
}
