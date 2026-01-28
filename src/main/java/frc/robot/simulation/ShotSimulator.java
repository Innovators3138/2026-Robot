package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;

public class ShotSimulator {
  private static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
  private static final Distance HUB_HEIGHT = Inches.of(72);
  private static final Distance HUB_OPENING_WIDTH = Inches.of(41.7);
  private static final Distance BLUE_HUB_X = Inches.of(158.6);
  private static final Distance BLUE_HUB_Y = Meters.of(4.03);
  private static final Distance FIELD_LENGTH = Meters.of(16.54);
  private static final Distance RED_HUB_X = FIELD_LENGTH.minus(BLUE_HUB_X);
  private static final Distance RED_HUB_Y = BLUE_HUB_Y;
  private static final Time MAX_BALL_LIFETIME = Seconds.of(5.0);
  private static final Time FIRE_INTERVAL = Milliseconds.of(200);
  public static final double EFFICIENCY = 0.9;
  private final List<SimulatedBall> activeBalls = new ArrayList<>();
  private final RobotContainer robotContainer;
  private int scoredCount = 0;
  private double timeSinceLastShot = 0;
  private final StructArrayPublisher<Pose3d> ballPosesPublisher;
  private final IntegerPublisher activeBallCountPublisher;
  private final IntegerPublisher scoredCountPublisher;
  private final FeederSubsystem feederSubsystem;

  private static class SimulatedBall {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double timeAlive;

    SimulatedBall(double x, double y, double z, double vx, double vy, double vz) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
      this.timeAlive = 0;
    }
  }

  public ShotSimulator(RobotContainer robotContainerm, FeederSubsystem feederSubsystem) {
    this.robotContainer = robotContainerm;
    this.feederSubsystem = feederSubsystem;

    var nt = NetworkTableInstance.getDefault();
    ballPosesPublisher =
        nt.getStructArrayTopic("Simulation/ShotSimulator/BallPoses", Pose3d.struct).publish();
    activeBallCountPublisher =
        nt.getIntegerTopic("Simulation/ShotSimulator/ActiveBallCount").publish();
    scoredCountPublisher = nt.getIntegerTopic("Simulation/ShotSimulator/ScoredCount").publish();
  }

  public void update(Time dt) {
    checkAndFire(dt);
    updateBalls(dt);
    logOutputs();
  }

  private void checkAndFire(Time dt) {
    var fireHeld = robotContainer.operatorXbox.rightTrigger(0.5).getAsBoolean();

    if (feederSubsystem.feeder.getSpeed().gte(RPM.of(1))) {
      timeSinceLastShot += dt.in(Seconds);
      var fireIntervalSeconds = FIRE_INTERVAL.in(Seconds);
      var angularVelocity = robotContainer.shooterSubsystem.getAngularVelocity();

      if (timeSinceLastShot >= fireIntervalSeconds && angularVelocity.gte(RPM.of(100))) {
        shoot();
        timeSinceLastShot = 0;
      }
    } else {
      timeSinceLastShot = FIRE_INTERVAL.in(Seconds);
    }
  }

  private void shoot() {
    var robotPose =
        robotContainer
            .swerveSubsystem
            .getSimulatedPose()
            .orElse(robotContainer.swerveSubsystem.getPose());
    var omegaRadPerSec = robotContainer.shooterSubsystem.getAngularVelocity().in(RadiansPerSecond);
    var wheelRadiusMeters = ShooterSubsystem.WHEEL_RADIUS.in(Meters);
    var exitVelocityMps = omegaRadPerSec * wheelRadiusMeters * EFFICIENCY;

    var robotHeading = robotPose.getRotation().getRadians();
    var launchAngleRad = ShooterSubsystem.LAUNCH_ANGLE.in(Radians);

    var shooterOffset = ShooterSubsystem.SHOOTER_OFFSET;
    var offsetX =
        shooterOffset.getX() * Math.cos(robotHeading)
            - shooterOffset.getY() * Math.sin(robotHeading);
    var offsetY =
        shooterOffset.getX() * Math.sin(robotHeading)
            + shooterOffset.getY() * Math.cos(robotHeading);

    var launchX = robotPose.getX() + offsetX;
    var launchY = robotPose.getY() + offsetY;
    var launchZ = shooterOffset.getZ();

    var horizontalVelocity = exitVelocityMps * Math.cos(launchAngleRad);
    var verticalVelocity = exitVelocityMps * Math.sin(launchAngleRad);

    var vx = horizontalVelocity * Math.cos(robotHeading);
    var vy = horizontalVelocity * Math.sin(robotHeading);
    var vz = verticalVelocity;

    activeBalls.add(new SimulatedBall(launchX, launchY, launchZ, vx, vy, vz));
  }

  private void updateBalls(Time dt) {
    var iterator = activeBalls.iterator();
    var dtSeconds = dt.in(Seconds);
    var gravityMps2 = GRAVITY.in(MetersPerSecondPerSecond);
    var maxLifetimeSeconds = MAX_BALL_LIFETIME.in(Seconds);
    var hubHeightMeters = HUB_HEIGHT.in(Meters);

    while (iterator.hasNext()) {
      var ball = iterator.next();
      var prevZ = ball.z;

      ball.vz -= gravityMps2 * dtSeconds;
      ball.x += ball.vx * dtSeconds;
      ball.y += ball.vy * dtSeconds;
      ball.z += ball.vz * dtSeconds;
      ball.timeAlive += dtSeconds;

      if (ball.z <= 0) {
        iterator.remove();
        continue;
      }

      if (checkHubScoring(ball, prevZ, hubHeightMeters)) {
        scoredCount++;
        iterator.remove();
        continue;
      }

      if (ball.timeAlive > maxLifetimeSeconds) {
        iterator.remove();
      }
    }
  }

  private boolean checkHubScoring(SimulatedBall ball, double prevZ, double hubHeightMeters) {
    var hubOpeningHalfWidth = HUB_OPENING_WIDTH.in(Meters) / 2.0;
    var blueHubX = BLUE_HUB_X.in(Meters);
    var blueHubY = BLUE_HUB_Y.in(Meters);
    var redHubX = RED_HUB_X.in(Meters);
    var redHubY = RED_HUB_Y.in(Meters);

    if (prevZ > hubHeightMeters && ball.z <= hubHeightMeters && ball.vz < 0) {
      var distToBlueHub =
          Math.sqrt(Math.pow(ball.x - blueHubX, 2) + Math.pow(ball.y - blueHubY, 2));
      if (distToBlueHub <= hubOpeningHalfWidth) {
        return true;
      }

      var distToRedHub = Math.sqrt(Math.pow(ball.x - redHubX, 2) + Math.pow(ball.y - redHubY, 2));
      if (distToRedHub <= hubOpeningHalfWidth) {
        return true;
      }
    }
    return false;
  }

  private void logOutputs() {
    var poses =
        activeBalls.stream()
            .map(ball -> new Pose3d(ball.x, ball.y, ball.z, new Rotation3d()))
            .toArray(Pose3d[]::new);

    ballPosesPublisher.set(poses);
    activeBallCountPublisher.set(activeBalls.size());
    scoredCountPublisher.set(scoredCount);
  }
}
