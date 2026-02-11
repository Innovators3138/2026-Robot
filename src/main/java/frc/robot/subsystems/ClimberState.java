package frc.robot.subsystems;

public enum ClimberState {
  Retracted(0, true),
  Extending(0, false),
  Extended(0, false),
  Climbing(1, true),
  Hold(0.5, true),
  Dismount(0, false),
  Retracting(0.046, true);
  public final double motorPower;
  public boolean isRatchetEngaged;

  ClimberState(double motorPower, boolean isRatchetEngaged) {
    this.motorPower = motorPower;
    this.isRatchetEngaged = isRatchetEngaged;
  }
}
