package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import java.io.IOException;

public class RobotContainer {

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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
    AutoCommands.startingChooser.setDefaultOption("Left Start", "Left Start");
    AutoCommands.startingChooser.addOption("Middle Start", "Middle Start");
    AutoCommands.startingChooser.addOption("Right Start", "Right Start");
    AutoCommands.intakeChooser.setDefaultOption("Left Intake", "Left Intake");
    AutoCommands.intakeChooser.addOption("Depot Intake", "Depot Intake");
    AutoCommands.intakeChooser.addOption("Right Intake", "Right Intake");
    AutoCommands.intakeChooser.addOption("No Intake", "No Intake");
    AutoCommands.startingChooser.addOption("Right Trench Start", "Right Trench Start");
    AutoCommands.startingChooser.addOption("Left Trench Start", "Left Trench Start");
    SmartDashboard.putData(AutoCommands.startingChooser);
    SmartDashboard.putData(AutoCommands.intakeChooser);

    NamedCommands.registerCommand(
        "fireCommand", FireCommand.fire(feederSubsystem, hotdogSubsystem, shooterSubsystem));

    AutoCommands.startingChooser.onChange(
        command -> {
          visionSubsystem.initializePose(new Pose3d(AutoCommands.getStartingPosition()));
          swerveSubsystem.resetOdometry(AutoCommands.getStartingPosition());
        });
  }

  private void configureBindings() {

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driverXbox));
    shooterSubsystem.setDefaultCommand(
        shooterSubsystem.setAngularVelocity(
            () -> {
              var setpoint = operatorXbox.getRawAxis(1);
              if ((setpoint) > -0.05) {
                return RPM.of(0);
              } else {
                return RPM.of(setpoint * -5000);
              }
            }));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.setAngularVelocity(RPM.of(0)));
    feederSubsystem.setDefaultCommand(feederSubsystem.setFeederAngularVelocity(RPM.of(0)));
    hotdogSubsystem.setDefaultCommand(hotdogSubsystem.setHotdogAngularVelocity(RPM.of(0)));
    operatorXbox
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  FireCommand.flywheelOffset += 2;
                }));
    operatorXbox
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  FireCommand.flywheelOffset -= 2;
                }));

    operatorXbox
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  swerveSubsystem.targetOffset -= 0.5;
                }));

    operatorXbox
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  swerveSubsystem.targetOffset += 0.5;
                }));

    operatorXbox
        .rightTrigger(0.5)
        .whileTrue(FireCommand.fire(feederSubsystem, hotdogSubsystem, shooterSubsystem));
    operatorXbox
        .rightTrigger()
        .onFalse(FireCommand.unjam(feederSubsystem, hotdogSubsystem).withTimeout(1));
    operatorXbox.a().toggleOnTrue(intakeSubsystem.setAngularVelocity(RPM.of(2500)));

    operatorXbox
        .rightTrigger()
        .and(operatorXbox.y())
        .whileTrue(FireCommand.unjam(feederSubsystem, hotdogSubsystem));

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(FireCommand.targetLock(shooterSubsystem, swerveSubsystem));

    operatorXbox
        .leftTrigger(0.5)
        .whileTrue(
            FireCommand.targetLock(shooterSubsystem, swerveSubsystem)
                .alongWith(swerveSubsystem.autoAimCommand()));

    operatorXbox.povDown().whileTrue(Commands.runOnce(() -> intakeSubsystem.openHopper()));
    operatorXbox.b().whileTrue(intakeSubsystem.setAngularVelocity(RPM.of(-900)));

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(swerveSubsystem.resetSimOdometry());
    }
  }

  public Command getAutonomousCommand()
      throws FileVersionException, IOException, org.json.simple.parser.ParseException {

    return AutoCommands.createAuto(this);
  }
}
