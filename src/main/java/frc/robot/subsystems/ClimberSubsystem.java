package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.PoundsForce;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Instantiates the subsystem */
public class ClimberSubsystem extends SubsystemBase {
  public static final Mass CLIMBING_MASS = Pounds.of(135);
  public static final Distance DRUM_RADIUS = Inches.of(0.5);
  public static final int RETRACTED_SENSOR_CHANNEL = 0;
  public static final int EXTENDED_SENSOR_CHANNEL = 1;
  public static final DCMotor MOTOR = DCMotor.getNEO(1);
  public static final double GEAR_RATIO = 48;
  public static final Distance BASE_LENGTH = Meters.of(0.3);
  public static final Distance MAX_EXTENSION = Meters.of(0.46);
  public static final Force SPRING_FORCE = PoundsForce.of(7);
  public static final Mass CLIMBER_MASS = Pounds.of(2);
  private final Servo ratchetServo;
  private ClimberState currentState;
  private final SparkMax sparkMax = new SparkMax(15, MotorType.kBrushless);
  private final DigitalInput extendedSensor = new DigitalInput(EXTENDED_SENSOR_CHANNEL);
  private final DigitalInput retractedSensor = new DigitalInput(RETRACTED_SENSOR_CHANNEL);
  private final BooleanPublisher extendedPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("Subsystem/Climber/Extended").publish();
  private final BooleanPublisher retractedPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("Subsystem/Climber/Retracted").publish();

  public ClimberSubsystem() {
    currentState = ClimberState.Retracted;
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.ratchetServo = new Servo(9);
    this.ratchetServo.set(1);
  }

  private void disengageRatchet() {
    ratchetServo.set(0);
  }

  private void engageRatchet() {
    ratchetServo.set(1);
  }

  public boolean isRatchetEngaged() {
    if (ratchetServo.get() < 0.5) {
      return false;

    } else {
      return true;
    }
  }

  public ClimberState getState() {
    return currentState;
  }

  public double getMotorSetpoint() {
    return sparkMax.get();
  }

  public Command extend() {

    return run(() -> updateState(ClimberState.Extending))
        .until(extendedSensor::get)
        .andThen(() -> updateState(ClimberState.Extended));
  }

  public Command retract() {

    return run(() -> updateState(ClimberState.Retracting))
        .until(retractedSensor::get)
        .andThen(() -> updateState(ClimberState.Retracted));
  }

  public Command climb() {
    return run(() -> updateState(ClimberState.Climbing))
        .withTimeout(0.25)
        .andThen(() -> updateState(ClimberState.Hold));
  }

  public Command dismount() {
    return run(() -> updateState(ClimberState.Dismount));
  }

  private void updateState(ClimberState updatedState) {
    sparkMax.set(updatedState.motorPower);
    currentState = updatedState;
    if (updatedState.isRatchetEngaged) {
      engageRatchet();
    } else {
      disengageRatchet();
    }
  }

  @Override
  public void periodic() {
    extendedPublisher.set(extendedSensor.get());
    retractedPublisher.set(retractedSensor.get());
  }
}
// Enum to represent the different states of the climber
