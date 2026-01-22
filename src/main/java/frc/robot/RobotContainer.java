package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();

    m_climberSubsystem.setDefaultCommand(m_climberSubsystem.setHeight(Meters.of(0)));
  }

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));

    operatorXbox.leftTrigger(0.5).whileTrue(shooterSubsystem.setAngularVelocity(RPM.of(2250)));
    operatorXbox.a().whileTrue(m_climberSubsystem.setHeight(Meters.of(0.5)));
    operatorXbox.b().whileTrue(m_climberSubsystem.setHeight(Meters.of(1)));
    operatorXbox.y().whileTrue(m_climberSubsystem.setHeight(Meters.of(0.3)));
    operatorXbox.x().whileTrue(m_climberSubsystem.setHeight(Meters.of(-0.3)));
  }
}
