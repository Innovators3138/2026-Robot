package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

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

public class ShooterSubsystem extends SubsystemBase {
  private SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 1, 0))
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax motor = new SparkMax(4, MotorType.kBrushless);

  private SmartMotorController motorController =
      new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motorController)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withUpperSoftLimit(RPM.of(1000))
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(flywheelConfig);

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  public AngularVelocity getAngularVelocity() {
    return shooter.getSpeed();
  }

  public Command setAngularVelocity(AngularVelocity angularVelocity) {
    return shooter.setSpeed(angularVelocity);
  }
}
