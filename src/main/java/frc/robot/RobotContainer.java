package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class RobotContainer {

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final HotdogSubsystem hotdogSubsystem = new HotdogSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem(shooterSubsystem);
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

  public RobotContainer() {

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    configureBindings();
    AutoCommands.climbChooser.setDefaultOption("Left Climb", "Left Climb");
    AutoCommands.climbChooser.addOption("Middle Climb", "Middle Climb");
    AutoCommands.climbChooser.addOption("Right Climb", "Right Climb");
    AutoCommands.intakeChooser.setDefaultOption("Left Intake", "Left Intake");
    AutoCommands.intakeChooser.addOption("Depot Intake", "Depot Intake");
    AutoCommands.intakeChooser.addOption("Right Intake", "Right Intake");
    SmartDashboard.putData(AutoCommands.climbChooser);
    SmartDashboard.putData(AutoCommands.intakeChooser);
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox, operatorXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));

    operatorXbox.rightTrigger().whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem));
    operatorXbox.a().toggleOnTrue(intakeSubsystem.setAngularVelocity(RPM.of(500)));
    operatorXbox.rightTrigger(0.5).whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem));
    // below are temporary for climber testing
    operatorXbox.povUp().onTrue(climberSubsystem.extend());
    operatorXbox.povDown().onTrue(climberSubsystem.retract());
    operatorXbox.x().onTrue(climberSubsystem.dismount());
    operatorXbox.y().onTrue(climberSubsystem.climb());
    operatorXbox.povLeft().onTrue(swerveSubsystem.sysIdDriveMotorCommand());

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(FireCommand.targetLock(shooterSubsystem, swerveSubsystem));
    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(swerveSubsystem.resetSimOdometry());
    }
  }

  /*  public Command getAutonomousCommand() {
    return AutoCommands.firstAuto(
        swerveSubsystem, shooterSubsystem, feederSubsystem, hotdogSubsystem);
  }*/
  /* public Command getAutonomousCommand() {
    return AutoCommands.pathfindToPathAuto(
        swerveSubsystem, shooterSubsystem, feederSubsystem, hotdogSubsystem);
  } */
  public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {

    return AutoCommands.createAuto(
        swerveSubsystem, feederSubsystem, shooterSubsystem, hotdogSubsystem);
  }
}
