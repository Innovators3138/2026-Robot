package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;

import java.lang.reflect.Array;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ClimberState;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;



public class ClimberSimulator {
  private final MechanismLigament2d ratchet;
  private final ClimberSubsystem climberSubsystem;
  private final Mechanism2d mech2d;
  private final MechanismLigament2d climberExtender;
  private final DIOSim extenderSensorSim = new DIOSim(ClimberSubsystem.EXTENDED_SENSOR_CHANNEL);
  private final DIOSim retractedSensorSim = new DIOSim(ClimberSubsystem.RETRACTED_SENSOR_CHANNEL);
  private final LoggedMechanism2d mech2d;
  private final LoggedMechanismLigament2d climberExtender;
  private final StructArrayPublisher climberPosePublisher;
  private final ArrayList<Pose3d> climberArrayPose;

  public ClimberSimulator(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.mech2d = new LoggedMechanism2d(40, 40);
    climberPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Simulation/ClimberSimulator/ClimberPose", Pose3d.struct).publish();
    var climberRoot = mech2d.getRoot("Climber", 20, 0);
    var climberLigament =
        new LoggedMechanismLigament2d(
            "Climber Ligament",
            ClimberSubsystem.BASE_LENGTH.in(Inches),
            90,
            6,
            new Color8Bit(Color.kAquamarine));
    climberRoot.append(climberLigament);
    this.climberExtender =
        new LoggedMechanismLigament2d("Climber Extender", 0, 0, 4, new Color8Bit(Color.kMaroon));
    climberLigament.append(climberExtender);
    var ratchetRoot = mech2d.getRoot("Ratchet", 18, 0);
    this.ratchet = new MechanismLigament2d("Ratchet Lagamen", 4, 0, 6, new Color8Bit(Color.kGreen));
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
    var updatedExtenderLength = distance.plus(Inches.of(climberExtender.getLength()));
    var clampedLength =
        Math.min(updatedExtenderLength.in(Inches), ClimberSubsystem.MAX_EXTENSION.in(Inches));
    climberExtender.setLength(clampedLength);
    if (climberExtender.getLength() <= 0) {
      retractedSensorSim.setValue(true);
      climberExtender.setLength(0);
    } else {
      retractedSensorSim.setValue(false);
    }
    if (climberExtender.getLength() >= ClimberSubsystem.MAX_EXTENSION.in(Inches)) {
      extenderSensorSim.setValue(true);
    } else {
      extenderSensorSim.setValue(false);
    }
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
