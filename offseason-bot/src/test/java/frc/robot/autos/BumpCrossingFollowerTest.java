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
      return new ChassisSpeeds(0.25, 0.5, 0.75);
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
      resetPose = currentPose;
      resetSpeeds = currentSpeeds;
    }
  }

  @Test
  void crossingVelocityFollowsRequestedDirectionForOutboundAndReturn() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    for (boolean red : new boolean[] {true, false}) {
      for (boolean outbound : new boolean[] {true, false}) {
        var tracker =
            new frc.robot.imu.BumpCrossingTracker(
                () -> 0.0, () -> 0.0, () -> 0.0, translation -> {});
        var direction = red == outbound ? Rotation2d.kZero : Rotation2d.k180deg;
        tracker.bumpCrossRequest(com.team581.autos.Point.ofRed(Pose2d.kZero), direction);
        var follower = new BumpCrossingFollower(new RecordingPathFollower(), tracker);
        var point = AutoPoint.ofBlue(Pose2d.kZero);
        var segment = com.team581.trailblazer.Trailblazer.segment(point).forever();
        var speeds =
            follower.calculateSpeeds(
                new ChassisSpeeds(), Pose2d.kZero, Pose2d.kZero, point, segment, 0);
        assertThat(speeds.vxMetersPerSecond).isEqualTo(red == outbound ? -4.0 : 4.0);
        assertThat(speeds.vyMetersPerSecond).isEqualTo(0.5);
        assertThat(speeds.omegaRadiansPerSecond).isEqualTo(0.75);
      }
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
