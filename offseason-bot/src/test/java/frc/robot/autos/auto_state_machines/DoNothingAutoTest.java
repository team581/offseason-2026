package frc.robot.autos.auto_state_machines;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class DoNothingAutoTest {
  @Test
  void startsCenteredWithHubOnStartingLineForBothAlliances() {
    var auto = new DoNothingAuto(null, null);

    var startingPoint = auto.getStartingPoint();
    var expectedRed =
        new Pose2d(
            BumpAutoStart.RED_STARTING_LINE_X + BumpAutoStart.HALF_BUMPER_METERS,
            FieldUtil.RED_HUB_CENTER.getY(),
            Rotation2d.k180deg);

    assertThat(startingPoint.redPose()).isEqualTo(expectedRed);
    assertThat(startingPoint.bluePose())
        .isEqualTo(
            new Pose2d(
                FieldUtil.FIELD_LENGTH_X - expectedRed.getX(),
                expectedRed.getY(),
                Rotation2d.kZero));
  }
}
