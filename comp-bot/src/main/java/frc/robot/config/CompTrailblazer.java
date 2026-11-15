package frc.robot.config;

import com.ctre.phoenix6.Utils;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.SpeedZone;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.runtime.TimeSource;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.CompTunerConstants;
import java.util.Optional;
import java.util.OptionalDouble;

public final class CompTrailblazer {
  static final double BUMPER_HALF_SIZE_METERS = Units.inchesToMeters(35.0 / 2);
  // Initial conservative estimates for the velocity-only testbed. Confirm on carpet.
  private static final EmpiricalDriveCalibration CALIBRATION =
      new EmpiricalDriveCalibration(4, 1.6, 0.070);
  private static boolean warmedUp = false;

  public static Trailblazer create() {
    return create(Utils::getCurrentTimeSeconds);
  }

  public static Trailblazer create(TimeSource time) {
    var frontLeft = CompTunerConstants.FrontLeft;
    var modules =
        ImmutableList.of(
            frontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight);
    var robot =
        new RobotDrivetrainSpec(
            61.2,
            5.20,
            frontLeft.WheelRadius,
            frontLeft.DriveMotorGearRatio,
            frontLeft.SteerMotorGearRatio,
            modules.stream()
                .map(module -> new Translation2d(module.LocationX, module.LocationY))
                .toList(),
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            frontLeft.SlipCurrent,
            0.20);
    ensureWarmedUp(robot);
    return new Trailblazer(time, robot, CALIBRATION, 0.020, Optional.of(CompAutoField.CLEARANCE));
  }

  /**
   * Pre-warms Trailblazer classloading and JIT paths during robot boot to prevent tick-0 spikes.
   */
  private static synchronized void ensureWarmedUp(RobotDrivetrainSpec robot) {
    if (warmedUp) {
      return;
    }
    warmedUp = true;
    double dt = 0.020;
    double[] syntheticClock = {1.0};
    var warmupPlanner =
        new Trailblazer(
            () -> syntheticClock[0], robot, CALIBRATION, dt, Optional.of(CompAutoField.CLEARANCE));
    Pose2d startPose = CompAutoField.RIGHT_TRENCH_START;
    Translation2d midPoint = CompAutoField.offset(startPose, -1.4, 0.0).getTranslation();
    Pose2d endPose = CompAutoField.offsetWithHeading(startPose, -2.2, 0.25, 165.0);
    var warmupSegment =
        Trailblazer.segment("roborio-warmup")
            .maxSpeed(3.0)
            .addZone(new SpeedZone(0.2, 1.2, 2.5))
            .through(midPoint, 0.25, goal -> goal.faceDegrees(175.0))
            .stopAt(endPose)
            .build();
    warmupPlanner.activate(warmupSegment);

    Pose2d pose = startPose;
    ChassisSpeeds speeds = new ChassisSpeeds();
    for (int i = 0; i <= 60; i++) {
      if (i == 60) {
        warmupPlanner.cancel();
      }
      var tickResult =
          warmupPlanner.update(
              new TickInput(
                  syntheticClock[0],
                  new RobotStateSnapshot(
                      syntheticClock[0],
                      pose,
                      speeds,
                      true,
                      false,
                      Optional.of(speeds),
                      OptionalDouble.empty(),
                      OptionalDouble.empty(),
                      true),
                  ImmutableMap.of(),
                  ImmutableMap.of(),
                  Optional.empty()));
      speeds = tickResult.commandedFieldSpeeds();
      pose =
          new Pose2d(
              pose.getX() + speeds.vxMetersPerSecond * dt,
              pose.getY() + speeds.vyMetersPerSecond * dt,
              pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
      syntheticClock[0] += dt;
    }
  }

  private CompTrailblazer() {}
}
