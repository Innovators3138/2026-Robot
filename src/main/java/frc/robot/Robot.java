package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// --- CORRECT 2026 REV IMPORTS ---
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits; // New for setSetpoint
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// These are the specific imports that fix your deprecation warnings:
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;   // Use these top-level imports
import com.revrobotics.PersistMode; // instead of SparkBase.PersistMode
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.*;

public class Robot extends TimedRobot {

    private static final int MASTER_ID = 3;
    private static final int FOLLOWER_ID = 2;

    private static final double INITIAL_kS = 0.3;
    private static final double INITIAL_kV = 0.002;
    private static final double INITIAL_kP = 0.0001;
    private static final double INITIAL_kD = 0.005;

    private final SparkFlex masterMotor = new SparkFlex(MASTER_ID, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(FOLLOWER_ID, MotorType.kBrushless);

    private final SparkClosedLoopController masterController = masterMotor.getClosedLoopController();
    private final RelativeEncoder masterEncoder = masterMotor.getEncoder();

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(INITIAL_kS, INITIAL_kV);

    private final SparkFlexConfig masterConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    private double lastP = INITIAL_kP;
    private double lastD = INITIAL_kD;

    private final SubsystemBase dummySubsystem = new SubsystemBase() {};

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(7),
                            Seconds.of(10)),
    new SysIdRoutine.Mechanism(
        // Tell SysId how to send Volts to your Spark Flex
        (voltage) -> masterMotor.setVoltage(voltage.in(Volts)),
        // Tell SysId how to record data (or let URCL handle it)
        null,
        dummySubsystem,
        "shooter"
    ));

    private final CommandXboxController operatorController = new CommandXboxController(1);

    @Override
    public void robotInit() {
        // --- 1. Master Config ---
        masterConfig.idleMode(IdleMode.kCoast);
        masterConfig.smartCurrentLimit(40);
        masterConfig.voltageCompensation(12.0);

        masterConfig.closedLoop
            .pid(INITIAL_kP, 0.0, INITIAL_kD)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            masterConfig.closedLoop.maxMotion
                .cruiseVelocity(6000)
                .maxAcceleration(10000) // RPM per second
                .allowedProfileError(10);

        // --- 2. Follower Config ---
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(MASTER_ID, true);

        // --- 3. Apply Configs (Using top-level ResetMode/PersistMode) ---
        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- 4. Dashboard ---
        SmartDashboard.putNumber("Tuning kS", INITIAL_kS);
        SmartDashboard.putNumber("Tuning kV", INITIAL_kV);
        SmartDashboard.putNumber("Tuning kP", INITIAL_kP);
        SmartDashboard.putNumber("Tuning kD", INITIAL_kD);
        SmartDashboard.putNumber("Target RPM", 0.0);

        configureButtonBindings();

    }

    private void configureButtonBindings() {
        // Example: Run shooter at 3000 RPM while holding the 'A' button
        operatorController.a().whileTrue(
            Commands.run(() -> {
                masterController.setSetpoint(3000, ControlType.kVelocity);
            })
            .finallyDo(() -> masterMotor.stopMotor()) // Stops when button is released
        );

        // Example: Trigger your SysId tests
        operatorController.y().whileTrue(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        operatorController.b().whileTrue(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        operatorController.a().whileTrue(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        operatorController.x().whileTrue(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Actual RPM", masterEncoder.getVelocity());

        //get bus voltage, applied voltage, and calculate actual volts
        double busVoltage = masterMotor.getBusVoltage();
        double appliedOutput = masterMotor.getAppliedOutput(); // Returns -1.0 to 1.0
        double actualVolts = busVoltage * appliedOutput;
        SmartDashboard.putNumber("Actual Volts", actualVolts);
    }

    @Override
    public void teleopPeriodic() {
        double kS = SmartDashboard.getNumber("Tuning kS", INITIAL_kS);
        double kV = SmartDashboard.getNumber("Tuning kV", INITIAL_kV);
        double kP = SmartDashboard.getNumber("Tuning kP", INITIAL_kP);
        double kD = SmartDashboard.getNumber("Tuning kD", 0);
        double targetRpm = SmartDashboard.getNumber("Target RPM", 0);

        // // Live Tuning
        // if (kP != lastP || kD != lastD) {
        //     masterConfig.closedLoop.pid(kP, 0.0, kD);
        //     // NoPersist is faster and safer for live tuning
        //     masterMotor.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //     lastP = kP;
        //     lastD = kD;
        // }

        // feedforward = new SimpleMotorFeedforward(kS, kV);
        // double ffVolts = feedforward.calculate(targetRpm);

        // // --- 4. Updated Command (setSetpoint) ---
        // masterController.setSetpoint(
        //     targetRpm,
        //     ControlType.kMAXMotionVelocityControl,
        //     ClosedLoopSlot.kSlot0,
        //     ffVolts,
        //     ArbFFUnits.kVoltage
        // );

        //SmartDashboard.putNumber("FF Volts", ffVolts);
    }

    @Override
    public void disabledInit() {
        masterMotor.stopMotor();
    }
}
