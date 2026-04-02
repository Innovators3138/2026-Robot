package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle candle;

  private final LEDRangeController leftAimController;
  private final LEDRangeController rightAimController;
  private final LEDRangeController speedController;

  private final RGBWColor green = new RGBWColor(0, 255, 0, 0);
  private final RGBWColor red = new RGBWColor(255, 0, 0, 0);
  private final RGBWColor black = new RGBWColor(0, 0, 0, 0);
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  public LEDSubsystem(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.candle = new CANdle(36);
    leftAimController =
        new LEDRangeController(19, 28, 45.0, false, "Subsystems/LED/LeftAim/LitLEDCount");
    rightAimController =
        new LEDRangeController(17, 8, 45.0, false, "Subsystems/LED/RightAim/LitLEDCount");
    speedController =
        new LEDRangeController(29, 48, 3000.0, true, "Subsystems/LED/Speed/LitLEDCount");
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

  @Override
  public void periodic() {
    candle.setControl(new SolidColor(18, 18).withColor(green));

    double hubAngle = swerveSubsystem.calculateHubAngle();
    leftAimController.update(candle, hubAngle);
    rightAimController.update(candle, -hubAngle);

    double speedSetpoint =
        shooterSubsystem.getAngularVelocitySetpoint().map(s -> s.in(RPM)).orElse(0.0);

    if (speedSetpoint == 0) {
      speedController.disable(candle);
    } else {
      double speedError = speedSetpoint - shooterSubsystem.getRealAngularVelocity().in(RPM);
      speedController.update(candle, speedError);
    }
  }
}
