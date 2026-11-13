package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class AimParameterUtilTest {
  private static void setAlliance(AllianceStationID alliance) {
    DriverStationSim.setAllianceStationId(alliance);
    DriverStationSim.notifyNewData();
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void fallbackFeedingFacesTheAllianceZoneForEitherAlliance() {
    var red = AimParameterUtil.getFallbackFeedingParameters();
    assertThat(red.goalAngle()).isEqualTo(180.0);
    assertThat(red.distance()).isEqualTo(8.0);
    assertThat(red.swerveTolerance()).isEqualTo(5.0);

    setAlliance(AllianceStationID.Blue1);
    var blue = AimParameterUtil.getFallbackFeedingParameters();
    assertThat(blue.goalAngle()).isEqualTo(0);
    assertThat(blue.distance()).isEqualTo(red.distance());
    assertThat(blue.swerveTolerance()).isEqualTo(red.swerveTolerance());
  }

  @EnumSource(FeedLocation.class)
  @ParameterizedTest
  void feedingAimsAtSelectedTargetWithWiderTolerance(FeedLocation location) {
    var pose =
        new Pose2d(
            location.getTranslation().minus(new Translation2d(3.0, 4.0)), Rotation2d.kCCW_90deg);

    var result = AimParameterUtil.getFeedingParameters(location, pose);

    assertThat(result.distance()).isCloseTo(5.0, offset(1e-9));
    assertThat(result.goalAngle()).isCloseTo(-126.86989764584402, offset(1e-9));
    assertThat(result.swerveTolerance())
        .isCloseTo(Math.toDegrees(Math.atan2(0.508, 5.0)), offset(1e-9));
  }

  @BeforeEach
  void resetAlliance() {
    setAlliance(AllianceStationID.Red1);
  }

  @Test
  void scoringHeadingIsFieldRelativeRegardlessOfCurrentRobotHeading() {
    var target = FieldUtil.HUB_POSE.getTranslation();
    var pose = new Pose2d(target.plus(new Translation2d(3.0, 4.0)), Rotation2d.kCW_90deg);

    var result = AimParameterUtil.getScoringParameters(pose);

    assertThat(result.distance()).isCloseTo(5.0, offset(1e-9));
    assertThat(result.goalAngle()).isCloseTo(53.13010235415598, offset(1e-9));
  }

  @Test
  void scoringUsesActualRangeAndBearingForRearFacingShooter() {
    var target = FieldUtil.HUB_POSE.getTranslation();
    var pose = new Pose2d(target.minus(new Translation2d(3.0, 4.0)), Rotation2d.kZero);

    var result = AimParameterUtil.getScoringParameters(pose);

    assertThat(result.distance()).isCloseTo(5.0, offset(1e-9));
    assertThat(result.goalAngle()).isCloseTo(-126.86989764584402, offset(1e-9));
    assertThat(result.swerveTolerance())
        .isCloseTo(Math.toDegrees(Math.atan2(0.254, 5.0)), offset(1e-9));
  }
}
