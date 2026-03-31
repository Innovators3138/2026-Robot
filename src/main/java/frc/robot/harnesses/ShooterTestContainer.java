package frc.robot.harnesses;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
public class ShooterTestContainer {
  public final HotdogSubsystem hotdogSubsystem = new HotdogSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);

  public final CommandXboxController operatorXbox = new CommandXboxController(1);
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem(shooterSubsystem, swerveSubsystem);
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  private AngularVelocity shooterSpeed = RPM.of(2500);

  ShooterTestContainer() {
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOriented(driverXbox, operatorXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(
            shooterSubsystem.setAngularVelocity(
                () -> {
                  return shooterSpeed;
                }));
    operatorXbox
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterSpeed = shooterSpeed.plus(RPM.of(50));
                }));
    operatorXbox
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterSpeed = shooterSpeed.minus(RPM.of(50));
                }));

    operatorXbox
        .rightTrigger(0.5)
        .whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem, shooterSubsystem));
  }
}
