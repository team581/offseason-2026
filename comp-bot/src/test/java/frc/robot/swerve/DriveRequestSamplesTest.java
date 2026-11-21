package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.stream.IntStream;
import org.junit.jupiter.api.Test;

final class DriveRequestSamplesTest {
  private static final int COLUMNS_PER_ROW = 20;
  private static final int BUFFER_CAPACITY = 128;

  private static void captureRow(DriveRequestSamples buffer, double time, long sequence) {
    buffer.capture(
        time, sequence, new ChassisSpeeds(), new ChassisSpeeds(), 0, 0, 0, 0, new Feedforwards(4));
  }

  @Test
  void dropsOverflowSamplesWhilePreservingOldestAndWrappingAfterDrain() {
    var buffer = new DriveRequestSamples("timestamped_velocity_and_force");
    IntStream.range(0, BUFFER_CAPACITY + 1).forEach(i -> captureRow(buffer, i, 42));

    assertEquals(1, buffer.dropped());
    var rows = buffer.drain();
    assertEquals(BUFFER_CAPACITY * COLUMNS_PER_ROW, rows.length);
    assertEquals(0, rows[0]);
    assertEquals(42, rows[1]);
    assertEquals(BUFFER_CAPACITY - 1, rows[(BUFFER_CAPACITY - 1) * COLUMNS_PER_ROW]);
    assertEquals(0, buffer.drain().length);

    captureRow(buffer, 999, 77);
    var wrapped = buffer.drain();
    assertEquals(COLUMNS_PER_ROW, wrapped.length);
    assertEquals(999, wrapped[0]);
    assertEquals(77, wrapped[1]);
    assertEquals(1, buffer.dropped());
  }

  @Test
  void preservesCaptureOrderAndEmptiesOnDrain() {
    var buffer = new DriveRequestSamples("timestamped_velocity_and_force");
    captureRow(buffer, 1.5, 7);
    captureRow(buffer, 2.5, 8);

    var rows = buffer.drain();
    assertEquals(2 * COLUMNS_PER_ROW, rows.length);
    assertEquals(1.5, rows[0]);
    assertEquals(7, rows[1]);
    assertEquals(2.5, rows[COLUMNS_PER_ROW]);
    assertEquals(8, rows[COLUMNS_PER_ROW + 1]);
    assertEquals(0, buffer.drain().length);
  }
}
