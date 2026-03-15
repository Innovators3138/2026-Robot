package frc.robot.harnesses;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTestContainer {
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final CommandXboxController operatorXbox = new CommandXboxController(1);
public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  ShooterTestContainer() {

    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(3000)));
    operatorXbox.a().whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(1500)));
    operatorXbox.b().whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(2000)));
    operatorXbox.x().whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(1000)));
    operatorXbox.y().whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(3000)));
  }
}
