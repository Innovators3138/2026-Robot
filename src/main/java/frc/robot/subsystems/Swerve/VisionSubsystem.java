package frc.robot.subsystems.Swerve;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final DoublePublisher targetYaw =
      NetworkTableInstance.getDefault()
          .getTable("AdvantageKit")
          .getDoubleTopic("tagYaw")
          .publish(null);
  private final IntegerPublisher apriltagPublisher =
      NetworkTableInstance.getDefault()
          .getTable("AdvantageKit")
          .getIntegerTopic("bestTag")
          .publish(null);
}
