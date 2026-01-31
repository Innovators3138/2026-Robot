package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final HotdogSubsystem hotdogSubsystem = new HotdogSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();

    climberSubsystem.setDefaultCommand(climberSubsystem.setHeight(Meters.of(0)));
  }

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox, operatorXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));

    operatorXbox.rightTrigger().whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem));
    operatorXbox.b().toggleOnTrue(climberSubsystem.setHeight(Meters.of(1)));
    operatorXbox.y().toggleOnTrue(climberSubsystem.toggleRatchet());
    operatorXbox.a().toggleOnTrue(intakeSubsystem.setAngularVelocity(RPM.of(1200)));
    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(FireCommand.targetLock(shooterSubsystem, swerveSubsystem));
  }
}
