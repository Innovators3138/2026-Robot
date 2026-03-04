package frc.robot.harnesses;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTestContainer {

 public final TestVisionSubsystem testVisionSubsystem = new TestVisionSubsystem();

  VisionTestContainer() {

  }
}
