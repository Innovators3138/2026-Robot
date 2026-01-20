package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
  public static final Distance WHEEL_DIAMETER = Inches.of(4);
  public static final Distance WHEEL_RADIUS = WHEEL_DIAMETER.div(2);
  public static final double EFFICIENCY = 0.75;

  public static final Distance SHOOTER_OFFSET_X = Meters.of(0.3);
  public static final Distance SHOOTER_OFFSET_Y = Meters.of(0);
  public static final Distance SHOOTER_OFFSET_Z = Meters.of(0.6);
  public static final Translation3d SHOOTER_OFFSET =
      new Translation3d(
          SHOOTER_OFFSET_X.in(Meters), SHOOTER_OFFSET_Y.in(Meters), SHOOTER_OFFSET_Z.in(Meters));

  public static final Angle LAUNCH_ANGLE = Degrees.of(70);

  private SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.25, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.25, 0))
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
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
          .withUpperSoftLimit(RPM.of(2500))
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
