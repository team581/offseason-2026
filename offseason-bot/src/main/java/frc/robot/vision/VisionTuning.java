package frc.robot.vision;

import com.team581.vision.spot.VisionProcessor;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

/**
 * Deviations fit to mttd Q11 and practice-field logs: repeat readings agree within 1 cm, and
 * cameras disagree by ~0.1-0.25 m at 3.6 m, so accuracy, not noise, sets the scale.
 */
public final class VisionTuning {
  private static final DoubleSubscriber TRUST =
      DogLog.tunable("Vision/Tuning/TrustworthyMeters", .5);
  private static final DoubleSubscriber FLOOR =
      DogLog.tunable("Vision/Tuning/TranslationFloorMeters", .02);
  private static final DoubleSubscriber RANGE =
      DogLog.tunable("Vision/Tuning/RangeSquaredCoefficient", .008);
  private static final DoubleSubscriber RESIDUAL =
      DogLog.tunable("Vision/Tuning/MaxResidualPixels", 20.0);
  private static final DoubleSubscriber INNOVATION =
      DogLog.tunable("Vision/Tuning/InnovationSigma", 10000.0);

  public static VisionProcessor.Policy policy() {
    return new VisionProcessor.Policy(
        positive(FLOOR.get(), .02),
        positive(RANGE.get(), .008),
        positive(RESIDUAL.get(), 2.),
        Math.toRadians(35),
        positive(INNOVATION.get(), 10.),
        Math.toRadians(15),
        .02);
  }

  public static double trustworthyMeters() {
    return Math.min(.99, positive(TRUST.get(), .5));
  }

  static double positive(double value, double fallback) {
    return Double.isFinite(value) && value > 0 ? value : fallback;
  }

  private VisionTuning() {}
}
