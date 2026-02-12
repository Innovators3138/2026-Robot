package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTestContainer {
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  ShooterTestContainer() {

    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));

    operatorXbox.a().whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(500)));
  }
}
