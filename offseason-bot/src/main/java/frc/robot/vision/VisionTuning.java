package frc.robot.vision;

import com.team581.vision.spot.VisionProcessor;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

/** Provisional bringup values; validate with measured camera geometry and on-robot logs. */
public final class VisionTuning {
  private static final DoubleSubscriber TRUST =
      DogLog.tunable("Vision/Tuning/TrustworthyMeters", .5);
  private static final DoubleSubscriber FLOOR =
      DogLog.tunable("Vision/Tuning/TranslationFloorMeters", .04);
  private static final DoubleSubscriber RANGE =
      DogLog.tunable("Vision/Tuning/RangeSquaredCoefficient", .015);
  private static final DoubleSubscriber RESIDUAL =
      DogLog.tunable("Vision/Tuning/MaxResidualPixels", 20.0);
  private static final DoubleSubscriber INNOVATION =
      DogLog.tunable("Vision/Tuning/InnovationSigma", 10.);

  public static VisionProcessor.Policy policy() {
    return new VisionProcessor.Policy(
        positive(FLOOR.get(), .04),
        positive(RANGE.get(), .015),
        positive(RESIDUAL.get(), 2.),
        Math.toRadians(15),
        Math.toRadians(35),
        positive(INNOVATION.get(), 10.),
        .5,
        Math.toRadians(30),
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
