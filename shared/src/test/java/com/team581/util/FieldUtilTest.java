package com.team581.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class FieldUtilTest {
  private static void setAlliance(AllianceStationID alliance) {
    DriverStationSim.setAllianceStationId(alliance);
    DriverStationSim.notifyNewData();
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  // Red alliance tests

  @Test
  void clampPoseInsideBlueAllianceZone() {
    setAlliance(AllianceStationID.Blue1);

    // Far left side of field, clearly inside blue alliance zone
    var robotPose = new Pose2d(2.417, 5.409, Rotation2d.kZero);

    var clamped = FieldUtil.clampPoseToAllianceZone(robotPose);

    assertThat(clamped).isEqualTo(robotPose);
  }

  @Test
  void clampPoseInsideRedAllianceZone() {
    setAlliance(AllianceStationID.Red1);

    // Far right side of field, clearly inside red alliance zone
    var robotPose = new Pose2d(14.340, 2.534, Rotation2d.kZero);
    var clamped = FieldUtil.clampPoseToAllianceZone(robotPose);

    assertThat(clamped).isEqualTo(robotPose);
  }

  // Blue alliance tests

  @Test
  void clampPoseOutsideBlueAllianceZone() {
    setAlliance(AllianceStationID.Blue1);

    // Middle of field, outside blue alliance zone
    var robotPose = new Pose2d(6.768, 0.842, Rotation2d.kZero);

    var clamped = FieldUtil.clampPoseToAllianceZone(robotPose);

    assertThat(clamped.getX()).isLessThan(4.628);
    assertThat(clamped.getY()).isCloseTo(robotPose.getY(), offset(0.01));
  }

  @Test
  void clampPoseOutsideRedAllianceZone() {
    setAlliance(AllianceStationID.Red1);

    // Middle of field, outside red alliance zone
    var robotPose = new Pose2d(10.758, 0.7509, Rotation2d.kZero);

    var clamped = FieldUtil.clampPoseToAllianceZone(robotPose);

    assertThat(clamped.getX()).isGreaterThan(12.527);
    assertThat(clamped.getY()).isCloseTo(robotPose.getY(), offset(0.01));
  }

  @Test
  void fallbackScorePointTrench() {
    setAlliance(AllianceStationID.Blue1);
    var blue1 = FieldUtil.getFallbackScorePoint();
    assertThat(blue1.getY()).isCloseTo(AprilTags.TAG_12.getY(), offset(1e-4));

    setAlliance(AllianceStationID.Blue3);
    var blue3 = FieldUtil.getFallbackScorePoint();
    assertThat(blue3.getY()).isCloseTo(AprilTags.TAG_7.getY(), offset(1e-4));

    setAlliance(AllianceStationID.Red1);
    var red1 = FieldUtil.getFallbackScorePoint();
    assertThat(red1.getY()).isCloseTo(AprilTags.TAG_17.getY(), offset(1e-4));

    setAlliance(AllianceStationID.Red3);
    var red3 = FieldUtil.getFallbackScorePoint();
    assertThat(red3.getY()).isCloseTo(AprilTags.TAG_22.getY(), offset(1e-4));
  }

  @Test
  void pathflipTest() {
    var input = new Pose2d(0, 0, Rotation2d.kZero);
    var expected =
        new Pose2d(FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, Rotation2d.k180deg);

    var result = FieldUtil.pathflip(input);

    assertThat(result).isEqualTo(expected);
  }

  @Test
  void towerFallbackScorePointBlueAlliance() {
    setAlliance(AllianceStationID.Blue1);
    var blueStation1Pose = FieldUtil.getTowerFallbackScorePoint();
    assertThat(blueStation1Pose.getX()).isCloseTo(Units.inchesToMeters(17.5) + 0.5, offset(1e-4));
    assertThat(blueStation1Pose.getY())
        .isCloseTo(FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(146.718750 - 17.5), offset(1e-4));
    assertThat(blueStation1Pose.getRotation()).isEqualTo(Rotation2d.k180deg);
    assertThat(FieldUtil.isScorePathObstructed(blueStation1Pose.getTranslation())).isFalse();

    setAlliance(AllianceStationID.Blue2);
    assertThat(FieldUtil.getTowerFallbackScorePoint()).isEqualTo(blueStation1Pose);

    setAlliance(AllianceStationID.Blue3);
    var blueStation3Pose = FieldUtil.getTowerFallbackScorePoint();
    assertThat(blueStation3Pose.getX()).isCloseTo(Units.inchesToMeters(17.5) + 0.5, offset(1e-4));
    assertThat(blueStation3Pose.getY())
        .isCloseTo(FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(193.718750 + 17.5), offset(1e-4));
    assertThat(blueStation3Pose.getRotation()).isEqualTo(Rotation2d.k180deg);
    assertThat(FieldUtil.isScorePathObstructed(blueStation3Pose.getTranslation())).isFalse();
  }

  @Test
  void towerFallbackScorePointRedAlliance() {
    setAlliance(AllianceStationID.Red1);
    var redStation1Pose = FieldUtil.getTowerFallbackScorePoint();
    assertThat(redStation1Pose.getX())
        .isCloseTo(FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(17.5) - 0.5, offset(1e-4));
    assertThat(redStation1Pose.getY())
        .isCloseTo(Units.inchesToMeters(146.718750 - 17.5), offset(1e-4));
    assertThat(redStation1Pose.getRotation()).isEqualTo(Rotation2d.kZero);
    assertThat(FieldUtil.isScorePathObstructed(redStation1Pose.getTranslation())).isFalse();

    setAlliance(AllianceStationID.Red2);
    assertThat(FieldUtil.getTowerFallbackScorePoint()).isEqualTo(redStation1Pose);

    setAlliance(AllianceStationID.Red3);
    var redStation3Pose = FieldUtil.getTowerFallbackScorePoint();
    assertThat(redStation3Pose.getX())
        .isCloseTo(FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(17.5) - 0.5, offset(1e-4));
    assertThat(redStation3Pose.getY())
        .isCloseTo(Units.inchesToMeters(193.718750 + 17.5), offset(1e-4));
    assertThat(redStation3Pose.getRotation()).isEqualTo(Rotation2d.kZero);
    assertThat(FieldUtil.isScorePathObstructed(redStation3Pose.getTranslation())).isFalse();
  }
}
