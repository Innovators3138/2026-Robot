package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.FireCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HotdogSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final HotdogSubsystem hotdogSubsystem = new HotdogSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem(shooterSubsystem, swerveSubsystem);
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  public final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
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
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));

    operatorXbox
        .rightTrigger()
        .whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem, shooterSubsystem));
    operatorXbox.a().toggleOnTrue(intakeSubsystem.setAngularVelocity(RPM.of(900)));

    operatorXbox
        .rightTrigger()
        .and(operatorXbox.y())
        .whileTrue(FireCommand.unjam(feederSubsystem, hotdogSubsystem, intakeSubsystem));

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(FireCommand.targetLock(shooterSubsystem, swerveSubsystem));

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(swerveSubsystem.resetSimOdometry());
    }
  }
}
