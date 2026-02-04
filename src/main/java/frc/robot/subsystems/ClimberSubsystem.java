package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.PoundsForce;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Instantiates the subsystem */
public class ClimberSubsystem extends SubsystemBase {
  public static final DCMotor MOTOR = DCMotor.getNEO(1);
  public static final double GEAR_RATIO = 48;
  public static final Distance BASE_LENGTH = Inches.of(12);
  public static final Distance MAX_EXTENSION = Inches.of(9);
  public static final Force SPRING_FORCE = PoundsForce.of(7);
  public static final Mass CLIMBER_MASS = Pounds.of(2);
  private final Servo ratchetServo;
  private ClimberState desiredState;
  private final SparkMax sparkMax = new SparkMax(9, MotorType.kBrushless);

  public ClimberSubsystem() {
    desiredState = ClimberState.Retracted;
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

  public Command setDesiredState(ClimberState state) {
    return runOnce(
        () -> {
          switch (state) {
            case Extended:
              disengageRatchet();
              desiredState = ClimberState.Extended;
              break;
            default:
              break;
          }
        });
  }
}
