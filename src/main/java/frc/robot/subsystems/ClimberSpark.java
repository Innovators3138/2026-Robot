package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class ClimberSpark {

  private static final int maximumRetries = 5;
  private SparkBaseConfig cfg;
  private final SparkBase motor;
  public SparkClosedLoopController pid;

  public ClimberSpark(SparkBase motor, SparkBaseConfig cfg, DCMotor motorType) {
    this.motor = motor;
    this.cfg = cfg;

    pid = motor.getClosedLoopController();

    configureSpark(motor::clearFaults);
  }

  public void configureSpark(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  public void updateConfig(SparkBaseConfig motorCfg, SparkBaseConfig cfgGiven) {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException(
          "Configuration changes cannot be applied while the robot is enabled.");
    }
    motorCfg.apply(cfgGiven);
    configureSpark(
        () ->
            motor.configure(
                motorCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void setVoltageCompensation(double nominalVoltage) {
    cfg.voltageCompensation(nominalVoltage);
  }

  public void setCurrentLimit(int currentLimit) {
    cfg.smartCurrentLimit(currentLimit);
  }

  public void setLoopRampRate(double rampRate) {
    cfg.closedLoopRampRate(rampRate).openLoopRampRate(rampRate);
  }

  public SparkBase getMotor() {
    return motor;
  }

  public void configurePIDWrapping(double minInput, double maxInput) {
    cfg.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(minInput, maxInput);
  }

  public void setMotorBrake(boolean isBrakeMode) {
    cfg.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setInverted(boolean inverted) {
    cfg.inverted(inverted);
  }

  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSpark(
        () -> {
          return motor.configure(
              cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
  }

  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  public void setReference(double setpoint, double feedforward) {
    configureSpark(
        () ->
            pid.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward));
  }

  public void setReference(double setpoint, double feedforward, double position) {
    setReference(setpoint, feedforward);
  }

  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public double getMotorTemperature() {
    return motor.getMotorTemperature();
  }
}
