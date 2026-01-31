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

public class IntakeSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(25), RotationsPerSecondPerSecond.of(35))
          .withSimClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(25), RotationsPerSecondPerSecond.of(35))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.5, 0))
          // Telemetry name and verbosity level
          .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax intake = new SparkMax(5, MotorType.kBrushless);

  private SmartMotorController intakeSmartMotorController =
      new SparkWrapper(intake, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(intakeSmartMotorController)
          .withDiameter(Inches.of(2))
          .withMass(Pounds.of(0.5))
          .withUpperSoftLimit(RotationsPerSecond.of(25))
          .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH);

  private FlyWheel Intake = new FlyWheel(flywheelConfig);

  @Override
  public void periodic() {
    Intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    Intake.simIterate();
  }

  public AngularVelocity getAngularVelocity() {
    return Intake.getSpeed();
  }

  public Command setAngularVelocity(AngularVelocity angularVelocity) {
    return Intake.setSpeed(angularVelocity);
  }
}
