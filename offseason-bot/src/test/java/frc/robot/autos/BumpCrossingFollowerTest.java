package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class BumpCrossingFollowerTest {
  private static final class RecordingPathFollower implements PathFollower {
    private Pose2d resetPose;
    private ChassisSpeeds resetSpeeds;

    @Override
    public ChassisSpeeds calculateSpeeds(
        ChassisSpeeds currentSpeeds,
        Pose2d currentPose,
        Pose2d targetPose,
        AutoPoint<?> currentPoint,
        AutoSegment segment,
        int currentPointIndex) {
      throw new UnsupportedOperationException();
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
      resetPose = currentPose;
      resetSpeeds = currentSpeeds;
    }
  }

  @Test
  void resetPreservesTranslationAndStartsAngularProfileFromRest() {
    var recordingFollower = new RecordingPathFollower();
    var follower = new BumpCrossingFollower(recordingFollower, null);
    var currentPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0));
    var currentSpeeds = new ChassisSpeeds(1.5, -2.5, -3.0);

    follower.reset(currentPose, currentSpeeds);

    assertThat(recordingFollower.resetPose).isSameAs(currentPose);
    assertThat(recordingFollower.resetSpeeds.vxMetersPerSecond)
        .isEqualTo(currentSpeeds.vxMetersPerSecond);
    assertThat(recordingFollower.resetSpeeds.vyMetersPerSecond)
        .isEqualTo(currentSpeeds.vyMetersPerSecond);
    assertThat(recordingFollower.resetSpeeds.omegaRadiansPerSecond).isEqualTo(0);
  }
}
