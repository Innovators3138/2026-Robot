package frc.robot.subsystems;

public enum ClimberState {
  Retracted(0),
  Extending(0),
  Extended(0),
  Climbing(1),
  Hold(0.5),
  Dismount(0),
  Retracting(1);
  public final double motorPower;
  ClimberState(double motorPower){
this.motorPower= motorPower

  }
}
