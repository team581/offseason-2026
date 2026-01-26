package com.team581.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.math.BaseTurretCalculator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class BaseTurretCalculatorTest {
  @Test
  void testCalculateTurretAimingAngle() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, 1.0);
    var robotRotation = Rotation2d.fromDegrees(0.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = 90.0;
    assertEquals(expected, actual, 1e-9);
  }
}
