package frc.robot.config;

import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);

  public static final BooleanSupplier BUMP_ASSIST = FeatureFlag.of("BumpAssist", true);
  public static final BooleanSupplier TRENCH_ASSIST = FeatureFlag.of("TrenchAssist", true);

  private FeatureFlags() {}
}
