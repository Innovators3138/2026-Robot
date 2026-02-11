package frc.robot.subsystems;

public enum ClimberState {
  Retracted(0, true),
  Extending(0, false),
  Extended(0, false),
  Climbing(0.58, true),
  Hold(0, true),
  Dismount(0.55, false),
  Retracting(0.046, true);
  public final double motorPower;
  public boolean isRatchetEngaged;

  ClimberState(double motorPower, boolean isRatchetEngaged) {
    this.motorPower = motorPower;
    this.isRatchetEngaged = isRatchetEngaged;
  }
}
