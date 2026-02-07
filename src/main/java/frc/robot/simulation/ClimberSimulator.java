package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSimulator {
  private final ClimberSubsystem climberSubsystem;
  private final Mechanism2d mech2d;
  private final MechanismLigament2d climberExtender;

  public ClimberSimulator(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.mech2d = new Mechanism2d(40, 40);
    var climberRoot = mech2d.getRoot("Climber", 20, 0);
    var climberLigament =
        new MechanismLigament2d(
            "Climber Ligament",
            ClimberSubsystem.BASE_LENGTH.in(Inches),
            90,
            6,
            new Color8Bit(Color.kAquamarine));
    climberRoot.append(climberLigament);
    this.climberExtender =
        new MechanismLigament2d("Climber Extender", 0, 0, 4, new Color8Bit(Color.kMaroon));
    climberLigament.append(climberExtender);
    SmartDashboard.putData("ClimberMechanism", mech2d);
  }

  public void update(Time dt) {
    var speed = calculateVelocity(dt);
    var distance = speed.times(dt);
    var updatedExtenderLength = distance.plus(Inches.of(climberExtender.getLength()));
    var clampedLength =
        Math.min(updatedExtenderLength.in(Inches), ClimberSubsystem.MAX_EXTENSION.in(Inches));
    climberExtender.setLength(clampedLength);
  }

  private LinearVelocity calculateVelocity(Time dt) {
    var springAcceleration = ClimberSubsystem.SPRING_FORCE.div(ClimberSubsystem.CLIMBER_MASS);
    var gravityAcceleration = MetersPerSecondPerSecond.of(-9.81);
    var netAcceleration = springAcceleration.minus(gravityAcceleration);
    return netAcceleration.times(dt);
  }
}
