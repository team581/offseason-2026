package com.team581.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.math.BaseTurretCalculator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

final class BaseTurretCalculatorTest {
  @Test
  void calculateSwerveTurretCompensationAngleNegative() {
    var actual =
        BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
            -45.0, Rotation2d.fromDegrees(45.0), -180.0, 180.0);
    var expected = -240.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateSwerveTurretCompensationAnglePositive() {
    var actual =
        BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
            45.0, Rotation2d.fromDegrees(45.0), -180.0, 180.0);
    var expected = 330.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegative() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, -1.0);
    var robotRotation = Rotation2d.fromDegrees(0.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = -90.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegativeWithNegativeRobotRotation() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, -1.0);
    var robotRotation = Rotation2d.fromDegrees(-45.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = -45.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAngleNegativeWithPositiveRobotRotation() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, -1.0);
    var robotRotation = Rotation2d.fromDegrees(45.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = -135.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositive() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, 1.0);
    var robotRotation = Rotation2d.fromDegrees(0.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = 90.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositiveWithNegativeRobotRotation() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, 1.0);
    var robotRotation = Rotation2d.fromDegrees(-45.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = 135.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void calculateTurretAimingAnglePositiveWithPositiveRobotRotation() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(0.0, 1.0);
    var robotRotation = Rotation2d.fromDegrees(45.0);

    var actual =
        BaseTurretCalculator.calculateTurretAimingAngle(
            robot, robotRotation, target, Transform2d.kZero);

    var expected = 45.0;
    assertEquals(expected, actual, 1e-9);
  }

  @Test
  void doesTurretHaveRoomNegative() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-90.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomNegativeAtLimit() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-170.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomPositive() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(90.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretHaveRoomPositiveAtLimit() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(170.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(true, actual);
  }

  @Test
  void doesTurretNotHaveRoomNegative() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(-178.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(false, actual);
  }

  @Test
  void doesTurretNotHaveRoomPositive() {
    var actual = BaseTurretCalculator.doesTurretHaveRoom(178.0, -180.0, 180.0, 10.0, 5.0);
    assertEquals(false, actual);
  }
}
