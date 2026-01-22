package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.FireCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox, operatorXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(FireCommand.targetLock(shooterSubsystem, swerveSubsystem));
  }
}
