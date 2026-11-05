package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

final class TurretCalculatorTest {
  @CsvSource({"0, -4.750, 8.025", "180, 4.750, -8.025", "270, 8.025, 4.750", "90, -8.025, -4.750"})
  @ParameterizedTest
  void cornerPositionVelocityAimAndToleranceUseRotatedOffset(
      double heading, double xInches, double yInches) {
    var robot = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(heading));
    double x = Units.inchesToMeters(xInches);
    double y = Units.inchesToMeters(yInches);
    var origin = new Translation2d(2.0 + x, 3.0 + y);
    assertThat(TurretCalculator.getTurretPose(robot).getTranslation().getDistance(origin))
        .isCloseTo(0.0, within(1e-9));
    var velocity =
        TurretCalculator.getTurretChassisSpeeds(new ChassisSpeeds(0.4, -0.7, 2.0), heading);
    assertThat(velocity.vxMetersPerSecond).isCloseTo(0.4 - 2.0 * y, within(1e-9));
    assertThat(velocity.vyMetersPerSecond).isCloseTo(-0.7 + 2.0 * x, within(1e-9));
    var goal = origin.plus(new Translation2d(3.0, 4.0));
    var expectedAngle = Rotation2d.fromRadians(Math.atan2(4.0, 3.0)).minus(robot.getRotation());
    var actualAngle =
        Rotation2d.fromDegrees(TurretCalculator.calculateTurretAimingAngle(robot, goal));
    assertThat(actualAngle.minus(expectedAngle).getRadians()).isCloseTo(0.0, within(1e-9));
    assertThat(TurretCalculator.getGoalCentricTurretTolerance(goal, robot, 0.1))
        .isCloseTo(Math.toDegrees(Math.atan2(0.1, 5.0)), within(1e-9));
  }

  @CsvSource({
    "-0.15, -0.9",
    "-0.499, -2.994",
    "-0.95, 0.3",
    "-12.15, -0.9",
    "0.0, 0.0",
    "0.15, 0.9",
    "0.499, 2.994",
    "0.95, -0.3",
    "12.15, 0.9"
  })
  @ParameterizedTest
  void homesToNearestMotorRevolution(double rotorDelta, double correctionDegrees) {
    double position =
        TurretCalculator.calculateHomedPosition(TurretConfig.MOTOR_ROTOR_CAL_OFFSET + rotorDelta);

    assertThat(Units.rotationsToDegrees(position))
        .isCloseTo(TurretConfig.HOMING_END_POSITION + correctionDegrees, within(1e-9));
  }
}
