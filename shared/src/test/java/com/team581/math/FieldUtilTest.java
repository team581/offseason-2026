package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import org.junit.jupiter.api.Test;

import com.team581.util.FieldUtil;

final class FieldUtilTest {
  private boolean insideAny(Translation2d robotTranslation) {
    return FieldUtil.TRENCH_BOXES.stream().anyMatch(trench -> trench.contains(robotTranslation));
  }

  @Test
  void outsideAllBoxesTest() {
    var robotTranslation =  new Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0));
    var insideBoxes = insideAny(robotTranslation);
    assertFalse(insideBoxes);
  }

  @Test
  void insideAnyBoxesTest() {
    var robotTranslation =  new Translation2d(Units.inchesToMeters(160.0), Units.inchesToMeters(20.0));
    var insideBoxes = insideAny(robotTranslation);
    assertTrue(insideBoxes);
  }

  @Test
  void insideRedRightBoxTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0), Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 0.508);
    var insideRedRight = FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertTrue(insideRedRight);
  }

  @Test
  void insideRedLeftBoxTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0), FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 7.562);
    var insideRedLeft = FieldUtil.RED_LEFT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertTrue(insideRedLeft);
  }

  @Test
  void insideBlueRightBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0), Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714,0.508);
    var insideBlueRight = FieldUtil.BLUE_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertTrue(insideBlueRight);
  }

  @Test
  void insideBlueLeftBoxTest() {
    // var robotTranslation = new Translation2d(FieldUtil.FIELD_LENGTH-Units.inchesToMeters(160.0), FieldUtil.FIELD_WIDTH-Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(12.4714, 7.562);
    var insideBlueLeft = FieldUtil.BLUE_LEFT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertTrue(insideBlueLeft);
  }

  @Test
  void insideOneAndAnyBoxesTest() {
    // var robotTranslation = new Translation2d(Units.inchesToMeters(160.0), Units.inchesToMeters(20.0));
    var robotTranslation = new Translation2d(4.064, 0.508);
    var insideBoxes = insideAny(robotTranslation);
    var insideRedRight = FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX.contains(robotTranslation);
    assertEquals(insideBoxes, insideRedRight);
  }
}
