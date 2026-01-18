package com.team581.math;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

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
    var insideRedRight = FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideRedRight).isTrue();
  }

  @Test
  void insideRedLeftBoxTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0),
    // FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 7.562);
    var insideRedLeft = FieldUtil.RED_LEFT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideRedLeft).isTrue();
  }

  @Test
  void insideBlueRightBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0),
    // Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714, 0.508);
    var insideBlueRight = FieldUtil.BLUE_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideBlueRight).isTrue();
  }

  @Test
  void insideBlueLeftBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0),
    // FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714, 7.562);
    var insideBlueLeft = FieldUtil.BLUE_LEFT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertThat(insideBlueLeft).isTrue();
  }

  @Test
  void insideOneAndAnyBoxesTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0),
    // Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 0.508);
    var insideBoxes = insideAny(robotTranslation);
    var insideRedRight = FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertEquals(insideBoxes, insideRedRight);
  }
}
