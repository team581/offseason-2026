package frc.robot.config;

import com.team581.autos.Point;
import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier INTEGRATION_TEST = FeatureFlag.of("IntegrationTest", false);

  public static final BooleanSupplier REGRESSION_MODEL = FeatureFlag.of("RegressionModel", false);

  public static final BooleanSupplier TOF_REGRESSION_MODEL =
      FeatureFlag.of("TofRegressionModel", true);

  public static final BooleanSupplier CANCEL_IN_PROGRESS_SHOT =
      FeatureFlag.of("CancelInProgressShot", true);

  public static final BooleanSupplier CLUSTER_MAP = FeatureFlag.of("ClusterMap", false);

  public static final BooleanSupplier CLAMPED_AUTO_POINTS = Point.CLAMPED_POINTS_FEATURE_FLAG;

  public static final BooleanSupplier OBSTRUCTION_GATING =
      FeatureFlag.of("ObstructionGating", false);

  private FeatureFlags() {}
}
