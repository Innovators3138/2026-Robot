package frc.robot.subsystems;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberSubsystem extends SubsystemBase {
  public final LoggedMechanism2d mechanism2d =
      new LoggedMechanism2d(3, 3, new Color8Bit(Color.kBlack));
  private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot("Climber Root", 1.5, 0);
  private final LoggedMechanismLigament2d climberMech2d =
      mechRoot2d.append(
          new LoggedMechanismLigament2d("Climber", ClimberConstants.CLIMBER_INITIAL_HEIGHT, null));

  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
