package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.lang.reflect.Array;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;



public class ClimberSimulator {
  private final ClimberSubsystem climberSubsystem;
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
    climberArrayPose = mech2d.generate3dMechanism();
    Pose3d[] poseArray = climberArrayPose.toArray(new Pose3d[0]);
    climberPosePublisher.set(poseArray);
    SmartDashboard.putData("ClimberMechanism", mech2d);

  }

  public void update(Time dt) {

    if (climberSubsystem.getRatchetPosition() > 0.5) {
      var downAcceleration = MetersPerSecond.of(-0.5);
      var speed = downAcceleration;
      var distance = speed.times(dt);
      var updatedExtenderLength = distance.plus(Inches.of(climberExtender.getLength()));
      var minLength = Math.max(updatedExtenderLength.in(Inches), 0);
      climberExtender.setLength(minLength);
    } else {
      var speed = calculateVelocity(dt);
      var distance = speed.times(dt);
      var updatedExtenderLength = distance.plus(Inches.of(climberExtender.getLength()));
      var clampedLength =
          Math.min(updatedExtenderLength.in(Inches), ClimberSubsystem.MAX_EXTENSION.in(Inches));
      climberExtender.setLength(clampedLength);
    }
  }

  private LinearVelocity calculateVelocity(Time dt) {
    var springAcceleration = ClimberSubsystem.SPRING_FORCE.div(ClimberSubsystem.CLIMBER_MASS);
    var gravityAcceleration = MetersPerSecondPerSecond.of(-9.81);
    var netAcceleration = springAcceleration.minus(gravityAcceleration);
    return netAcceleration.times(dt);
  }
}
