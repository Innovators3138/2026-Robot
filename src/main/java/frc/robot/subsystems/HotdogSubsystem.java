package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

public class HotdogSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig HsmcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(3), DegreesPerSecondPerSecond.of(3))
          .withSimClosedLoopController(
              0, 0, 0, RotationsPerSecond.of(9), DegreesPerSecondPerSecond.of(6))
          // Feedforward Constants
          .withFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.8, 0))
          // Telemetry name and verbosity level
          .withTelemetry("HotdogMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example GearBox.fromReductionStages(3,4) is the same as
          // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your
          // motor.
          // You could also use .withGearing(12) which does the same thing.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax hotdog = new SparkMax(8, MotorType.kBrushless);

  private SmartMotorController hotdogSmartMotorController =
      new SparkWrapper(hotdog, DCMotor.getNEO(1), HsmcConfig);

  private final FlyWheelConfig hflywheelConfig =
      new FlyWheelConfig(hotdogSmartMotorController)
          .withDiameter(Inches.of(3.50))
          .withMass(Pounds.of(0.5))
          .withUpperSoftLimit(RPM.of(180))
          .withTelemetry("HotdogMech", TelemetryVerbosity.HIGH);

  private FlyWheel Hotdog = new FlyWheel(hflywheelConfig);

  @Override
  public void periodic() {

    Hotdog.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {

    Hotdog.simIterate();
  }

  public AngularVelocity hotdogGetAngularVelocity() {
    return Hotdog.getSpeed();
  }

  public Command setHotdogAngularVelocity(AngularVelocity angularVelocity) {
    return Hotdog.setSpeed(angularVelocity);
  }
}
