package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;

public class Robot extends TimedRobot {

    // --- Constants ---
    private static final int MASTER_ID = 3;
    private static final int FOLLOWER_ID = 2;

    // CTRE Magnetic Encoder is usually 4096 ticks per revolution.
    // Change this if you are using a different encoder (e.g., Grayhill, E4T).
    private static final double ENCODER_TICKS_PER_REV = 4096.0;

    // --- Hardware ---
    private final WPI_TalonSRX masterMotor = new WPI_TalonSRX(MASTER_ID);
    private final WPI_TalonSRX followerMotor = new WPI_TalonSRX(FOLLOWER_ID);

    @Override
    public void robotInit() {
        // 1. Factory default to ensure consistent starting state
        masterMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        // 2. Set Neutral Mode (Brake or Coast)
        masterMotor.setNeutralMode(NeutralMode.Coast);
        followerMotor.setNeutralMode(NeutralMode.Coast);

        // 3. Configure Follower
        // This tells the follower to mimic the master's output
        followerMotor.follow(masterMotor);

        // 4. Invert the follower
        // InvertType.OpposeMaster ensures it drives opposite to the master
        followerMotor.setInverted(InvertType.OpposeMaster);

        // Optional: Invert master if positive voltage moves it "backwards"
        masterMotor.setInverted(false);

        // 5. Initialize Dashboard values
        // We put a default value so you can edit it immediately in AdvantageScope
        SmartDashboard.putNumber("Target Voltage", 0.0);
    }

    @Override
    public void robotPeriodic() {
        // --- RPM Calculation ---
        // Get velocity in "raw units per 100ms"
        double rawVelocity = masterMotor.getSelectedSensorVelocity();

        // Conversion:
        // (ticks/100ms * 10) = ticks/second
        // (ticks/second * 60) = ticks/minute
        // (ticks/minute) / ticks_per_rev = RPM
        double rpm = (rawVelocity * 10.0 * 60.0) / ENCODER_TICKS_PER_REV;

        // Send to NetworkTables (viewable in AdvantageScope)
        SmartDashboard.putNumber("Motor RPM", rpm);
        SmartDashboard.putNumber("Raw Velocity", rawVelocity);
    }

    @Override
    public void teleopPeriodic() {

      System.out.println("In teleop periodic");
        // 1. Get the target voltage from SmartDashboard/AdvantageScope
        // Default to 0.0 if communication is lost
        double targetVolts = SmartDashboard.getNumber("Target Voltage", 0.0);

        // 2. Safety clamp (RobotController usually provides battery voltage,
        // but 12.0 is a safe constant for clamping)
        if (targetVolts > 12.0) targetVolts = 12.0;
        if (targetVolts < -12.0) targetVolts = -12.0;

        System.out.println("Setting voltage to: " + targetVolts);

        // 3. Command the master motor
        // The follower will automatically output the inverted voltage
        masterMotor.setVoltage(targetVolts);
    }

    @Override
    public void disabledInit() {
        // Safety: Stop motors when disabled
        masterMotor.stopMotor();
    }
}