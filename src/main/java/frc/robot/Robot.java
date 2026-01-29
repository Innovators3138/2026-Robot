package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// --- NEW 2026 REV IMPORTS ---
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int MASTER_ID = 3;
    private static final int FOLLOWER_ID = 2;

    // --- Tuning Parameters ---
    // REMEMBER: Spark Flex Hardware PID is duty-cycle based (-1 to 1).
    // Scale your old Volts-based kP by 1/12.
    private static final double INITIAL_kS = 0.1;
    private static final double INITIAL_kV = 0.002;
    private static final double INITIAL_kP = 0.0001;
    private static final double INITIAL_kD = 0.0;

    // --- Hardware ---
    private final SparkFlex masterMotor = new SparkFlex(MASTER_ID, MotorType.kBrushless);
    private final SparkFlex followerMotor = new SparkFlex(FOLLOWER_ID, MotorType.kBrushless);

    // --- Control Objects ---
    private final SparkClosedLoopController masterController = masterMotor.getClosedLoopController();
    private final RelativeEncoder masterEncoder = masterMotor.getEncoder();

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(INITIAL_kS, INITIAL_kV);

    // --- Configuration State ---
    // We keep a copy of the config object to modify and re-apply during tuning
    private final SparkFlexConfig masterConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    // Trackers for Live Tuning (to avoid spamming CAN bus)
    private double lastP = INITIAL_kP;
    private double lastD = INITIAL_kD;

    @Override
    public void robotInit() {
        // --- 1. Setup Master Config ---
        masterConfig.idleMode(IdleMode.kCoast);
        masterConfig.smartCurrentLimit(40); // Always good practice
        masterConfig.voltageCompensation(12.0);

        // Configure PID in the config object
        masterConfig.closedLoop
            .pid(INITIAL_kP, 0.0, INITIAL_kD)
            .velocityFF(0.0) // We use Arbitrary Feedforward, so set internal kF to 0
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // --- 2. Setup Follower Config ---
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(40);
        followerConfig.voltageCompensation(12.0);

        // Configure Follower Mode
        // "true" means inverted relative to master
        followerConfig.follow(MASTER_ID, true);

        // --- 3. Apply Configs ---
        // configure() takes (config, ResetMode, PersistMode)
        // ResetSafeParameters: Clears old settings to ensure clean state
        // PersistParameters: Saves to flash (like burnFlash used to)
        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- 4. Dashboard ---
        SmartDashboard.putNumber("Tuning kS", INITIAL_kS);
        SmartDashboard.putNumber("Tuning kV", INITIAL_kV);
        SmartDashboard.putNumber("Tuning kP", INITIAL_kP);
        SmartDashboard.putNumber("Tuning derivative", INITIAL_kD);
        SmartDashboard.putNumber("Target RPM", 0.0);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Actual RPM", masterEncoder.getVelocity());
        SmartDashboard.putNumber("Applied Output", masterMotor.getAppliedOutput());
    }

    @Override
    public void teleopPeriodic() {
        // 1. Get Live Tuning Values
        double kS = SmartDashboard.getNumber("Tuning kS", INITIAL_kS);
        double kV = SmartDashboard.getNumber("Tuning kV", INITIAL_kV);
        double kP = SmartDashboard.getNumber("Tuning kP", INITIAL_kP);
        double kD = SmartDashboard.getNumber("Tuning derivative", 0);
        double targetRpm = SmartDashboard.getNumber("Target RPM", 0);

        // 2. Live PID Tuning Logic
        // In the new API, we must update the config object and re-apply it.
        // We MUST check if values changed, otherwise we will crash the CAN bus
        // by writing to flash 50 times a second.
        if (kP != lastP || kD != lastD) {
            masterConfig.closedLoop.pid(kP, 0.0, kD);

            // Apply ONLY to RAM (NoPersist) for tuning to be fast and save flash life
            masterMotor.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastP = kP;
            lastD = kD;
            System.out.println("Updated PID: P=" + kP + " D=" + kD);
        }

        // 3. Calculate Feedforward (SimpleMotorFeedforward is still valid and useful)
        feedforward = new SimpleMotorFeedforward(kS, kV);
        double ffVolts = feedforward.calculate(targetRpm);

        // 4. Command the Motor
        // setReference is deprecated/removed in favor of setReference (with different args) or strict use of controller
        // The new API uses the controller object specifically:
        masterController.setReference(
            targetRpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0, // Use default PID slot 0
            ffVolts // Arbitrary Feedforward (Volts because we enabled voltage comp)
        );

        SmartDashboard.putNumber("FF Volts", ffVolts);
    }

    @Override
    public void disabledInit() {
        masterMotor.stopMotor();
    }
}