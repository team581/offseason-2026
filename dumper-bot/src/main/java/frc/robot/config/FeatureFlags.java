package frc.robot.config;

import java.util.function.BooleanSupplier;

import com.team581.config.FeatureFlag;

public class FeatureFlags {
  public static final BooleanSupplier SHOOT_ON_THE_MOVE = FeatureFlag.of("ShootOnTheMove", false);
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);
  public static final BooleanSupplier CLUSTER_MAP = FeatureFlag.of("ClusterMapEnabled", true);
  public static final BooleanSupplier INTAKE_WALL_SNAPS = FeatureFlag.of("IntakeWallSnaps", false);
  public static final BooleanSupplier INTAKE_DIRECTIONAL_SNAPS =
      FeatureFlag.of("IntakeDirectionalSnaps", false);

  private FeatureFlags() {}
}
