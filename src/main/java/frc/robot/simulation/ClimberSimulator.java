package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ClimberState;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.ArrayList;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberSimulator {
  private final LoggedMechanismLigament2d ratchet;
  private final ClimberSubsystem climberSubsystem;

  private final LoggedMechanism2d mech2d;
  private final LoggedMechanismLigament2d climberExtender;
  private final StructArrayPublisher climberPosePublisher;
  private final ArrayList<Pose3d> climberArrayPose;
  private final DIOSim extenderSensorSim = new DIOSim(ClimberSubsystem.EXTENDED_SENSOR_CHANNEL);
  private final DIOSim retractedSensorSim = new DIOSim(ClimberSubsystem.RETRACTED_SENSOR_CHANNEL);
  private LoggedMechanismRoot2d extenderRoot;
  private double extenderY;
  private final SparkMaxSim motorSim;

  public ClimberSimulator(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.mech2d = new LoggedMechanism2d(1, 1);
    motorSim = new SparkMaxSim(climberSubsystem.sparkMax, ClimberSubsystem.MOTOR);
    climberPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Simulation/ClimberSimulator/ClimberPose", Pose3d.struct)
            .publish();
    var climberRoot = mech2d.getRoot("Climber", 0.14, 0.23);

    var climberLigament =
        new LoggedMechanismLigament2d(
            "Climber Ligament",
            ClimberSubsystem.BASE_LENGTH.in(Meters),
            90,
            6,
            new Color8Bit(Color.kAquamarine));
    climberRoot.append(climberLigament);
    extenderY = 0.23;
    this.extenderRoot = mech2d.getRoot("Extender", 0.14, extenderY);
    this.climberExtender =
        new LoggedMechanismLigament2d("Climber Extender", 0.3, 90, 4, new Color8Bit(Color.kMaroon));
    extenderRoot.append(climberExtender);
    var ratchetRoot = mech2d.getRoot("Ratchet", 0.1, 0.23);
    this.ratchet =
        new LoggedMechanismLigament2d("Ratchet Lagamen", 0.1, 0, 6, new Color8Bit(Color.kGreen));
    ratchetRoot.append(ratchet);
    climberArrayPose = mech2d.generate3dMechanism();
    Pose3d[] poseArray = climberArrayPose.toArray(new Pose3d[0]);
    climberPosePublisher.set(poseArray);
    SmartDashboard.putData("ClimberMechanism", mech2d);
  }

  public void update(Time dt) {

    var speed = calculateVelocity(dt);
    if (climberSubsystem.isRatchetEngaged()) {
      ratchet.setColor(new Color8Bit(Color.kGreen));
    } else {
      ratchet.setColor(new Color8Bit(Color.kRed));
    }
    if (climberSubsystem.isRatchetEngaged() && speed.gt(MetersPerSecond.zero())) {
      speed = MetersPerSecond.zero();
    }
    var distance = speed.times(dt);
    var updatedExtenderLength = distance.plus(Meters.of(extenderY));
    var clampedLength =
        Math.min(updatedExtenderLength.in(Meters), ClimberSubsystem.MAX_EXTENSION.in(Meter));
    extenderRoot.setPosition(0.14, clampedLength);
    extenderY = clampedLength;
    if (extenderY <= 0.23) {
      retractedSensorSim.setValue(true);
      extenderRoot.setPosition(0.14, 0.23);
      extenderY = 0.23;
    } else {
      retractedSensorSim.setValue(false);
    }
    if (extenderY >= ClimberSubsystem.MAX_EXTENSION.in(Inches)) {
      extenderSensorSim.setValue(true);
    } else {
      extenderSensorSim.setValue(false);
    }
    var simRPM = climberSubsystem.sparkMax.getAppliedOutput() * 5676;
    motorSim.iterate(simRPM, RobotController.getBatteryVoltage(), dt.in(Seconds));
  }

  private LinearVelocity calculateVelocity(Time dt) {
    var gravityAcceleration = MetersPerSecondPerSecond.of(-9.81);

    var gravityForce = getMass().times(gravityAcceleration);
    var motorTorque =
        ClimberSubsystem.MOTOR.KtNMPerAmp
            * ClimberSubsystem.MOTOR.nominalVoltageVolts
            * climberSubsystem.getMotorSetpoint()
            * ClimberSubsystem.GEAR_RATIO;
    var motorForce = NewtonMeters.of(motorTorque).div(ClimberSubsystem.DRUM_RADIUS);
    var netForce = ClimberSubsystem.SPRING_FORCE.minus(gravityForce).minus(motorForce);
    var netAcceleration = netForce.div(ClimberSubsystem.CLIMBER_MASS);
    return netAcceleration.times(dt);
  }

  private Mass getMass() {
    if (climberSubsystem.getState() == ClimberState.Climbing
        || climberSubsystem.getState() == ClimberState.Hold
        || climberSubsystem.getState() == ClimberState.Dismount) {
      return ClimberSubsystem.CLIMBING_MASS;
    } else {
      return ClimberSubsystem.CLIMBER_MASS;
    }
  }
}
