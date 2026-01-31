package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/* Instantiates the subsystem */
public class ClimberSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
          .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH)
          .withClosedLoopController(
              0.0, 0.0, 0.0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          .withSimClosedLoopController(
              0.0, 0.0, 0.0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withFeedforward(new ElevatorFeedforward(0, 0, 0.1))
          .withSimFeedforward(new ElevatorFeedforward(0.02, 0.61, 1.46))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Second.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private Servo ratchetServo = new Servo(19);

  private SparkMax spark = new SparkMax(23, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  public Command toggleRatchet() {
    return runOnce(
        () -> {
          if (ratchetServo.get() < 0.5) {
            ratchetServo.set(1.0);
            climb.set(-1);
          } else {
            ratchetServo.set(0.0);
          }
        });
  }

  public double getRatchetPosition() {
    return ratchetServo.get();
  }

  private ElevatorConfig climbConfig =
      new ElevatorConfig(sparkSmartMotorController)
          .withStartingHeight(Meters.of(0.5))
          .withHardLimits(Meters.of(0), Meters.of(3))
          .withTelemetry("Climber", TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(16));

  private ElevatorConfig normalonfig =
      new ElevatorConfig(sparkSmartMotorController)
          .withStartingHeight(Meters.of(0.5))
          .withHardLimits(Meters.of(0), Meters.of(3))
          .withTelemetry("Climber", TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(16));

  private Elevator climb = new Elevator(climbConfig);
  private Elevator normal = new Elevator(normalonfig);

  public Command setHeight(Distance height) {
    return normal.setHeight(height);
  }

  public Command set(double dutycycle) {
    return normal.set(dutycycle);
  }

  public Command sysId() {
    return normal.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    climb.updateTelemetry();

    SmartDashboard.putNumber("Climber/RatchetPosition", getRatchetPosition());
  }

  @Override
  public void simulationPeriodic() {
    climb.simIterate();
  }
}
