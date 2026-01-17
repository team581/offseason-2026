package frc.robot.config;

import java.util.function.BooleanSupplier;

import com.team581.config.FeatureFlag;

public class FeatureFlags {
  public static final BooleanSupplier SHOOT_ON_THE_MOVE =
      FeatureFlag.of("ShootOnTheMove", false);

  private FeatureFlags() {}
}
