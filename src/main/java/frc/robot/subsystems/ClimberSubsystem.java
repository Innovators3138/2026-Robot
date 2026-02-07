package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.PoundsForce;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Instantiates the subsystem */
public class ClimberSubsystem extends SubsystemBase {
  public static final Distance BASE_LENGTH = Inches.of(12);
  public static final Distance MAX_EXTENSION = Inches.of(9);
  public static final Force SPRING_FORCE = PoundsForce.of(7);
  public static final Mass CLIMBER_MASS = Pounds.of(2);
  private final Servo ratchetServo;

  public ClimberSubsystem() {
    this.ratchetServo = new Servo(9);
    this.ratchetServo.set(1);
  }

  public Command toggleRatchet() {
    return runOnce(
        () -> {
          if (isRatchetEngaged()) {
            ratchetServo.set(0);

          } else {
            ratchetServo.set(1);
          }
        });
  }

  public boolean isRatchetEngaged() {
    if (ratchetServo.get() < 0.5) {
      return false;

    } else {
      return true;
    }
  }
}
