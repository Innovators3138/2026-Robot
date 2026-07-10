package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
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
  InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  public static final Distance WHEEL_DIAMETER = Inches.of(4);
  public static final Distance WHEEL_RADIUS = WHEEL_DIAMETER.div(2);

  public static final Distance SHOOTER_OFFSET_X = Meters.of(-0.4);
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
              0.01, 0, 0.0, RotationsPerSecond.of(50), RotationsPerSecondPerSecond.of(80))
          .withSimClosedLoopController(
              0.05, 0, 0, RotationsPerSecond.of(50), RotationsPerSecondPerSecond.of(80))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.1075, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.125, 0))
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(70));

  private SparkMax shooterMotorLeaderLeft = new SparkMax(21, MotorType.kBrushless);
  private SparkMax shooterMotorFollowerLeft = new SparkMax(20, MotorType.kBrushless);
  private SparkMax shooterMotorFollowerRightA = new SparkMax(18, MotorType.kBrushless);
  private SparkMax shooterMotorFollowerRightB = new SparkMax(19, MotorType.kBrushless);

  private SmartMotorController motorController =
      new SparkWrapper(shooterMotorLeaderLeft, DCMotor.getNEO(2), motorConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motorController)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withUpperSoftLimit(RPM.of(500))
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(flywheelConfig);

  public ShooterSubsystem() {
    var followerConfigLeft = new SparkMaxConfig();
    followerConfigLeft.follow(21, false);
    followerConfigLeft.idleMode(IdleMode.kCoast);
    shooterMotorFollowerLeft.configure(
        followerConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var followerConfigRight = new SparkMaxConfig();
    followerConfigRight.follow(21, true);
    followerConfigRight.idleMode(IdleMode.kCoast);
    shooterMotorFollowerRightA.configure(
        followerConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotorFollowerRightB.configure(
        followerConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
    Logger.recordOutput("Subsystems/shooter/angularvelocity", getRealAngularVelocity());
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  public AngularVelocity getRealAngularVelocity() {
    return shooter.getSpeed();
  }

  public Optional<AngularVelocity> getAngularVelocitySetpoint() {
    return shooter.getMotorController().getMechanismSetpointVelocity();
  }

  public Command setAngularVelocity(AngularVelocity angularVelocity) {
    return shooter.setSpeed(angularVelocity);
  }

  public Command setAngularVelocity(Supplier<AngularVelocity> angularVelocity) {
    return shooter.setSpeed(angularVelocity);
  }
}
