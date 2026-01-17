package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int MASTER_ID = 3;
    private static final int FOLLOWER_ID = 2;
    private static final double ENCODER_TICKS_PER_REV = 4096.0;

    // --- Tuning Parameters (Initial Guesses) ---
    // kS: Voltage to overcome static friction (Start small, e.g., 0.5)
    // kV: Voltage per RPM (Approx 12V / MaxRPM). If max is 6000, kV ~ 0.002
    // kP: Proportional Error Correction. Start at 0.
    private static final double INITIAL_kS = 1.0;
    private static final double INITIAL_kV = 0.002;
    private static final double INITIAL_kP = 0.02;
    private static final double INITIAL_kD = 0.0001;  //pid derivative

    // --- Hardware ---
    private final WPI_TalonSRX masterMotor = new WPI_TalonSRX(MASTER_ID);
    private final WPI_TalonSRX followerMotor = new WPI_TalonSRX(FOLLOWER_ID);

    // --- Control Objects ---
    private final PIDController pid = new PIDController(INITIAL_kP, 0, 0);
    // Feedforward calculates the voltage required to sustain a specific velocity
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(INITIAL_kS, INITIAL_kV);

    @Override
    public void robotInit() {
        // 1. Configure Motors
        masterMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        masterMotor.setNeutralMode(NeutralMode.Coast);
        followerMotor.setNeutralMode(NeutralMode.Coast);

        // 2. Setup Follower
        followerMotor.follow(masterMotor);
        followerMotor.setInverted(InvertType.OpposeMaster);
        masterMotor.setInverted(false);

        // 3. Initialize Dashboard for Live Tuning
        SmartDashboard.putNumber("Tuning kS", INITIAL_kS);
        SmartDashboard.putNumber("Tuning kV", INITIAL_kV);
        SmartDashboard.putNumber("Tuning kP", INITIAL_kP);
        SmartDashboard.putNumber("Tuning derivative", INITIAL_kD);
        SmartDashboard.putNumber("Target RPM", 0.0);
    }

    @Override
    public void robotPeriodic() {
        // --- RPM Calculation ---
        // Get raw velocity (ticks per 100ms)
        double rawVelocity = masterMotor.getSelectedSensorVelocity();
        // Convert to RPM
        double currentRpm = (rawVelocity * 10.0 * 60.0) / ENCODER_TICKS_PER_REV;

        SmartDashboard.putNumber("Actual RPM", currentRpm);
    }

    @Override
    public void teleopPeriodic() {
        // 1. Get Live Tuning Values
        double kS = SmartDashboard.getNumber("Tuning kS", INITIAL_kS);
        double kV = SmartDashboard.getNumber("Tuning kV", INITIAL_kV);
        double kP = SmartDashboard.getNumber("Tuning kP", INITIAL_kP);
        double kD = SmartDashboard.getNumber("Tuning derivative", 0);
        double targetRpm = SmartDashboard.getNumber("Target RPM", 0);

        // 2. Update Controller Settings
        if (pid.getP() != kP) pid.setP(kP);
        if (pid.getD() != kD) pid.setD(kD);

        // (Re-creating this object every loop is fine for prototyping)
        feedforward = new SimpleMotorFeedforward(kS, kV);

        // 3. Calculate Outputs
        // Feedforward: "I predict I need X volts to go this fast"
        double ffVolts = feedforward.calculate(targetRpm);

        // PID: "I am off by Y amount, so add Z volts to fix it"
        double pidVolts = pid.calculate(SmartDashboard.getNumber("Actual RPM", 0), targetRpm);

        // 4. Combine and Clamp
        double totalVolts = ffVolts + pidVolts;

        // Safety Clamp (Safe range for 12V battery)
        if (totalVolts > 12.0) totalVolts = 12.0;
        if (totalVolts < 0) totalVolts = 0.0;

        // 5. Apply to Motor
        masterMotor.setVoltage(totalVolts);

        // 6. Debug Data
        SmartDashboard.putNumber("Applied Volts", totalVolts);
        SmartDashboard.putNumber("FF Volts", ffVolts);
        SmartDashboard.putNumber("PID Volts", pidVolts);
    }

    @Override
    public void disabledInit() {
        masterMotor.stopMotor();
    }
}