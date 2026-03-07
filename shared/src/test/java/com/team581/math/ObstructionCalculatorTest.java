package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.Test;

final class ObstructionCalculatorTest {
  private static final ObstructionCalculator CALCULATOR =
      ObstructionCalculator.fromTranslations(
          List.of(
              new Translation2d(0, 0),
              new Translation2d(2, 0),
              new Translation2d(2, 2),
              new Translation2d(0, 2)));

  @Test
  void multipleObstructions() {
    var calculator =
        ObstructionCalculator.fromTranslations(
            List.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(1, 1),
                new Translation2d(0, 1)),
            List.of(
                new Translation2d(5, 5),
                new Translation2d(6, 5),
                new Translation2d(6, 6),
                new Translation2d(5, 6)));

    assertTrue(calculator.isObstructed(new Translation2d(0.5, 0.5)));
    assertTrue(calculator.isObstructed(new Translation2d(5.5, 5.5)));
    assertFalse(calculator.isObstructed(new Translation2d(3, 3)));
  }

  @Test
  void pointInsideObstructionIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(1, 1)));
  }

  @Test
  void pointOnCornerIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(0, 0)));
  }

  @Test
  void pointOnEdgeIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(1, 0)));
  }

  @Test
  void pointOutsideObstructionIsNotObstructed() {
    assertFalse(CALCULATOR.isObstructed(new Translation2d(5, 5)));
  }

  @Test
  void segmentBetweenObstructionsIsNotObstructed() {
    var calculator =
        ObstructionCalculator.fromTranslations(
            List.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(1, 1),
                new Translation2d(0, 1)),
            List.of(
                new Translation2d(3, 0),
                new Translation2d(4, 0),
                new Translation2d(4, 1),
                new Translation2d(3, 1)));

    assertFalse(calculator.isObstructed(new Translation2d(1.5, 0.5), new Translation2d(2.5, 0.5)));
  }

  @Test
  void segmentEndingAtEdgeIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(-1, 1), new Translation2d(0, 1)));
  }

  @Test
  void segmentEntirelyInsideObstructionIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(0.5, 0.5), new Translation2d(1.5, 1.5)));
  }

  @Test
  void segmentOutsideObstructionIsNotObstructed() {
    assertFalse(CALCULATOR.isObstructed(new Translation2d(3, 3), new Translation2d(5, 5)));
  }

  @Test
  void segmentStartingInsideIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(1, 1), new Translation2d(5, 5)));
  }

  @Test
  void segmentThroughObstructionIsObstructed() {
    assertTrue(CALCULATOR.isObstructed(new Translation2d(-1, 1), new Translation2d(3, 1)));
  }
}
