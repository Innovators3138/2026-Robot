package frc.robot.harnesses;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTestContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  SwerveTestContainer() {
    var testTranslation = new Translation2d(0.25, 0.0);
    driverXbox.start().onTrue(swerveSubsystem.resetOdometry());
    driverXbox.a().onTrue(swerveSubsystem.drive(testTranslation, 0, false).withTimeout(1));
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobotOriented(driverXbox, operatorXbox));
  }
}
