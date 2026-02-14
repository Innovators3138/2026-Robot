package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    autoChooser.setDefaultOption("Basic Auto", "Basic Auto");
    autoChooser.addOption("Pathfind to Path Auto", "Pathfind to Path Auto");
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    configureBindings();

    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox, operatorXbox));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.setAngularVelocity(RPM.of(0)));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));
    ledSubsystem.setDefaultCommand(ledSubsystem.setToGreen());

    operatorXbox.rightTrigger().whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem));
    operatorXbox.a().toggleOnTrue(intakeSubsystem.setAngularVelocity(RPM.of(500)));
    operatorXbox.rightTrigger(0.5).whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem));
    // below are temporary for climber testing
    operatorXbox.povUp().onTrue(climberSubsystem.extend());
    operatorXbox.povDown().onTrue(climberSubsystem.retract());
    operatorXbox.x().onTrue(climberSubsystem.dismount());
    operatorXbox.y().onTrue(climberSubsystem.climb());

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
  public Command getAutonomousCommand() {
    var selectedAuto = autoChooser.getSelected();
    if (selectedAuto == null) {
      return AutoCommands.basicAuto(
          swerveSubsystem, shooterSubsystem, feederSubsystem, hotdogSubsystem, climberSubsystem);
    }
    if (selectedAuto.equals("Basic Auto")) {
      return AutoCommands.basicAuto(
          swerveSubsystem, shooterSubsystem, feederSubsystem, hotdogSubsystem, climberSubsystem);
    }
    if (selectedAuto.equals("Pathfind to Path Auto")) {
      return AutoCommands.pathfindToPathAuto(
          swerveSubsystem, shooterSubsystem, feederSubsystem, hotdogSubsystem);
    }
    throw new RuntimeException("Unknown auto selected: " + selectedAuto);
  }
}
