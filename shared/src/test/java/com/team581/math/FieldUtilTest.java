package com.team581.math;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

final class FieldUtilTest {
  private boolean insideAny(Translation2d robotTranslation) {
    return FieldUtil.TRENCH_BOXES.stream().anyMatch(trench -> trench.contains(robotTranslation));
  }

  @Test
  void outsideAllBoxesTest() {
    var robotTranslation = new Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0));
    var insideBoxes = insideAny(robotTranslation);
    assertThat(insideBoxes).isFalse();
  }

  @Test
  void insideAnyBoxesTest() {
    var robotTranslation =
        new Translation2d(Units.inchesToMeters(160.0), Units.inchesToMeters(20.0));
    var insideBoxes = insideAny(robotTranslation);
    assertThat(insideBoxes).isTrue();
  }

  @Test
  void insideRedRightBoxTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0),
    // Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 0.508);
    var insideRedRight = FieldUtil.RED_OUTPOST_SIDE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideRedRight).isTrue();
  }

  @Test
  void insideRedLeftBoxTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0),
    // FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 7.562);
    var insideRedLeft = FieldUtil.RED_DEPOT_SIDE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideRedLeft).isTrue();
  }

  @Test
  void insideBlueRightBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0),
    // Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714, 0.508);
    var insideBlueRight = FieldUtil.BLUE_DEPOT_SIDE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideBlueRight).isTrue();
  }

  @Test
  void insideBlueLeftBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0),
    // FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714, 7.562);
    var insideBlueLeft = FieldUtil.BLUE_OUTPOST_SIDE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideBlueLeft).isTrue();
  }

  @Test
  void insideOneAndAnyBoxesTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0),
    // Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 0.508);
    var insideBoxes = insideAny(robotTranslation);
    var insideRedRight = FieldUtil.RED_OUTPOST_SIDE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideBoxes).isEqualTo(insideRedRight);
  }

  // Alliance Zone Tests
  @Test
  void insideRedAllianceZone() {
    var robotTranslation = new Translation2d(1.0, 1.0);
    var isInside = FieldUtil.RED_ALLIANCE_ZONE.contains(robotTranslation);
    assertThat(isInside).isTrue();
  }

  @Test
  void insideBlueAllianceZone() {
    var robotTranslation = new Translation2d(15.0, 1.0);
    var isInside = FieldUtil.BLUE_ALLIANCE_ZONE.contains(robotTranslation);
    assertThat(isInside).isTrue();
  }

  @Test
  void outsideRedAllianceZone() {
    var robotTranslation = new Translation2d(8.0, 1.0);
    var isInside = FieldUtil.RED_ALLIANCE_ZONE.contains(robotTranslation);
    assertThat(isInside).isFalse();
  }

  @Test
  void outsideBlueAllianceZone() {
    var robotTranslation = new Translation2d(8.0, 1.0);
    var isInside = FieldUtil.BLUE_ALLIANCE_ZONE.contains(robotTranslation);
    assertThat(isInside).isFalse();
  }

  @Test
  void nearestLegalScoringPoseNotInsideRed() {
    var robotTranslation = new Translation2d(8.0, 1.0);
    var expected = new Translation2d(FieldUtil.RED_STARTING_LINE_X, 1.0);
    var actual = FieldUtil.RED_ALLIANCE_ZONE.nearest(robotTranslation);
    assertThat(expected).isEqualTo(actual);
  }

  @Test
  void nearestLegalScoringPoseNotInsideBlue() {
    var robotTranslation = new Translation2d(8.0, 1.0);
    var expected = new Translation2d(FieldUtil.BLUE_STARTING_LINE_X, 1.0);
    var actual = FieldUtil.BLUE_ALLIANCE_ZONE.nearest(robotTranslation);
    assertThat(expected).isEqualTo(actual);
  }
}
