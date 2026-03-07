package frc.robot.harnesses;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestContainer {
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  ClimberTestContainer() {
    operatorXbox.povUp().onTrue(climberSubsystem.extend());
    operatorXbox.povDown().onTrue(climberSubsystem.retract());
    operatorXbox.x().onTrue(climberSubsystem.dismount());
    operatorXbox.y().onTrue(climberSubsystem.climb());
  }
}
