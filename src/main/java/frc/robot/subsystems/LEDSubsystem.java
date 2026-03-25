package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;

  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor red = new RGBWColor(255, 0, 0, 0);
  private final RGBWColor black = new RGBWColor(0, 0, 0, 0);
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  public LEDSubsystem(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.candle = new CANdle(36);
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
    var led = LEDConstants.LED_SPEED_NUMBER;
    var level = (int) (percentage * led);
    RGBWColor color;
    var startOffset = LEDConstants.LED_ANGLE_NUMBER + 7;
    if (percentage > 1) {
      color = red;
      level = 20;
    } else if (level > 20) {
      level = 20;
      color = green;

    } else {
      color = green;
    }
    candle.setControl(new SolidColor(startOffset + 1, startOffset + level + 7).withColor(color));
    Logger.recordOutput("Subsystems/Ledsubsystem/FlywheelLEDs", level);
  }

  private void shooterAimLEDs() {
    var hubAngle = (int) swerveSubsystem.calculateHubAngle();
    var deadZoneLimit = LEDConstants.LED_DEADZONE / 2;
    var startOffset = 7;
    int start;
    int end;
    int amount;
    int multiplier = LEDConstants.LED_MULTIPLIER;
    int increment = LEDConstants.LED_INCREMENT;
    // make sure LED_ANGLE_NUMBER is an odd number so there is a middle value
    int frontHalfEnd = LEDConstants.LED_ANGLE_NUMBER / 2;
    int middle = frontHalfEnd + startOffset + 1;
    int backHalfStart = middle + 1;
    int realAmount;
    if (hubAngle < 45 && hubAngle > -45) {

      if (hubAngle >= -deadZoneLimit && hubAngle <= deadZoneLimit) {
        start = middle;
        end = middle;

      } else if (hubAngle < -deadZoneLimit) {

        amount = Math.abs(hubAngle) / increment;
        realAmount = amount * multiplier;
        if (amount > 5) {
          amount = 5;
        }
        start = middle - realAmount;
        end = frontHalfEnd;
      } else if (hubAngle > deadZoneLimit) {
        amount = Math.abs(hubAngle) / increment;
        if (amount > 5) {
          amount = 5;
        }
        start = backHalfStart;
        end = amount * multiplier + middle;
      } else {
        start = 0;
        end = 0;
      }
    } else if (hubAngle > 45) {
      start = 8;
      end = frontHalfEnd + 7;

    } else {
      start = backHalfStart;
      end = middle + frontHalfEnd;
    }
    candle.setControl(new SolidColor(start, end).withColor(green));
  }

  public void testLEDs() {
    candle.setControl(new SolidColor(0, 7).withColor(green));
  }

  @Override
  public void periodic() {
    candle.setControl(new SolidColor(8, LEDConstants.TOTAL_LEDS).withColor(black));
    postTheColors();
    shooterAimLEDs();
  }
}
