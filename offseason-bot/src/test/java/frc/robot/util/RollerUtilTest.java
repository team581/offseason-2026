package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import edu.wpi.first.math.util.Units;
import frc.robot.conveyor.ConveyorConfig;
import frc.robot.feeder.FeederConfig;
import frc.robot.funneler.FunnelerConfig;
import frc.robot.intake.IntakeConfig;
import org.junit.jupiter.api.Test;

final class RollerUtilTest {
  @Test
  void convertsOneRollerRevolutionPerSecondToCircumference() {
    assertThat(RollerUtil.surfaceSpeedMetersPerSecond(1.0, 0.1, 1.0))
        .isCloseTo(Math.PI * 0.1, within(1e-12));
  }

  @Test
  void conveyorUsesFloorGearTrainAndInchDiameterFromSpec() {
    // At 60 motor RPS, independently apply every non-unity stage from the sheet.
    double expectedSpeed =
        60.0
            * (14.0 / 60.0)
            * (16.0 / 18.0)
            * (16.0 / 20.0)
            * (17.0 / 20.0)
            * (17.0 / 20.0)
            * Math.PI
            * 0.0381;
    assertThat(
            RollerUtil.surfaceSpeedMetersPerSecond(
                60.0, ConveyorConfig.ROLLER_DIAMETER_METERS, ConveyorConfig.MOTOR_TO_ROLLER_RATIO))
        .isCloseTo(expectedSpeed, within(1e-12));
  }

  @Test
  void hardwareReductionsProduceOneRollerRevolutionPerSecond() {
    double circumference = Math.PI * Units.inchesToMeters(2.0);
    assertThat(
            RollerUtil.surfaceSpeedMetersPerSecond(
                20.0 / 12.0,
                IntakeConfig.LEFT_ROLLER_DIAMETER_METERS,
                IntakeConfig.MOTOR_TO_ROLLER_RATIO))
        .isCloseTo(circumference, within(1e-12));
    assertThat(
            RollerUtil.surfaceSpeedMetersPerSecond(
                20.0 / 12.0,
                IntakeConfig.RIGHT_ROLLER_DIAMETER_METERS,
                IntakeConfig.MOTOR_TO_ROLLER_RATIO))
        .isCloseTo(circumference, within(1e-12));
    assertThat(
            RollerUtil.surfaceSpeedMetersPerSecond(
                28.0 / 18.0,
                FeederConfig.ROLLER_DIAMETER_METERS,
                FeederConfig.MOTOR_TO_ROLLER_RATIO))
        .isCloseTo(circumference, within(1e-12));
    assertThat(
            RollerUtil.surfaceSpeedMetersPerSecond(
                30.0 / 16.0,
                FunnelerConfig.ROLLER_DIAMETER_METERS,
                FunnelerConfig.MOTOR_TO_ROLLER_RATIO))
        .isCloseTo(circumference, within(1e-12));
  }

  @Test
  void missingHardwareDimensionsRemainUnknownEvenWhenStopped() {
    assertThat(RollerUtil.surfaceSpeedMetersPerSecond(0.0, Double.NaN, 1.0)).isNaN();
    assertThat(RollerUtil.surfaceSpeedMetersPerSecond(20.0, 0.1, Double.NaN)).isNaN();
  }

  @Test
  void preservesStoppedAndReverseMotion() {
    assertThat(RollerUtil.surfaceSpeedMetersPerSecond(0.0, 0.1, 2.0)).isEqualTo(0);
    assertThat(RollerUtil.surfaceSpeedMetersPerSecond(-20.0, 0.1, 2.0))
        .isCloseTo(-Math.PI, within(1e-12));
  }
}
