package com.team581.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    var robotTranslation = new Translation2d(2.417, 5.409);
    var clamped = FieldUtil.clampPoseToAllianceZone(robotTranslation);

    assertThat(clamped).isEqualTo(robotTranslation);
  }

  @Test
  void clampPoseInsideRedAllianceZone() {
    setAlliance(AllianceStationID.Red1);

    // Far right side of field, clearly inside red alliance zone
    var robotTranslation = new Translation2d(14.340, 2.534);
    var clamped = FieldUtil.clampPoseToAllianceZone(robotTranslation);

    assertThat(clamped).isEqualTo(robotTranslation);
  }

  // Blue alliance tests

  @Test
  void clampPoseOutsideBlueAllianceZone() {
    setAlliance(AllianceStationID.Blue1);

    // Middle of field, outside blue alliance zone
    var robotTranslation = new Translation2d(6.768, 0.842);
    var clamped = FieldUtil.clampPoseToAllianceZone(robotTranslation);

    assertThat(clamped.getX()).isLessThan(4.628);
    assertThat(clamped.getY()).isCloseTo(robotTranslation.getY(), offset(0.01));
  }

  @Test
  void clampPoseOutsideRedAllianceZone() {
    setAlliance(AllianceStationID.Red1);

    // Middle of field, outside red alliance zone
    var robotTranslation = new Translation2d(10.758, 0.7509);
    var clamped = FieldUtil.clampPoseToAllianceZone(robotTranslation);

    assertThat(clamped.getX()).isGreaterThan(12.527);
    assertThat(clamped.getY()).isCloseTo(robotTranslation.getY(), offset(0.01));
  }

  @Test
  void pathflipTest() {
    var input = new Pose2d(0, 0, Rotation2d.kZero);
    var expected =
        new Pose2d(FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, Rotation2d.k180deg);

    var result = FieldUtil.pathflip(input);

    assertThat(result).isEqualTo(expected);
  }
}
