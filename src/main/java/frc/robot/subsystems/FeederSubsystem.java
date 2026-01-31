package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FeederSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(40), RotationsPerSecondPerSecond.of(50))
          .withSimClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(40), RotationsPerSecondPerSecond.of(50))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.25, 0))
          // Telemetry name and verbosity level
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax feederMotor = new SparkMax(6, MotorType.kBrushless);

  private SmartMotorController feederSmartMotorController =
      new SparkWrapper(feederMotor, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(feederSmartMotorController)
          .withDiameter(Inches.of(1))
          .withMass(Pounds.of(0.5))
          .withUpperSoftLimit(RotationsPerSecond.of(40))
          .withTelemetry("FeederMech", TelemetryVerbosity.HIGH);

  public FlyWheel feeder = new FlyWheel(flywheelConfig);

  @Override
  public void periodic() {
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    feeder.simIterate();
  }

  public AngularVelocity feederGetAngularVelocity() {
    return feeder.getSpeed();
  }

  public Command setFeederAngularVelocity(AngularVelocity angularVelocity) {
    return feeder.setSpeed(angularVelocity);
  }
}
