package frc.robot.config;

import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier SHOOT_ON_THE_MOVE = FeatureFlag.of("ShootOnTheMove", false);
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);
  public static final BooleanSupplier CLUSTER_MAP = FeatureFlag.of("ClusterMapEnabled", true);
  public static final BooleanSupplier INTAKE_WALL_SNAPS = FeatureFlag.of("IntakeWallSnaps", true);

  private FeatureFlags() {}
}
