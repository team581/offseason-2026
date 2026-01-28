package frc.robot.config;

import com.team581.config.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier SHOOT_ON_THE_MOVE = FeatureFlag.of("ShootOnTheMove", false);
  public static final BooleanSupplier VISION_CAMERA_POSITION_COMPENSATION =
      FeatureFlag.of("CameraPositionComp", true);
  public static final BooleanSupplier VISION_TURRET_POSITION_COMPENSATION =
      FeatureFlag.of("TurretPositionComp", true);
  public static final BooleanSupplier VISION_TURRET_ANGLE_COMPENSATION =
      FeatureFlag.of("TurretAngleComp", true);
  public static final BooleanSupplier VISION_HUB_TAGS_FILTER =
      FeatureFlag.of("OnlyUseHubTags", true);
  public static final BooleanSupplier TRENCH_ASSIST = FeatureFlag.of("TrenchAssist", false);
  public static final BooleanSupplier BUMP_ASSIST = FeatureFlag.of("BumpAssist", false);

  public static final BooleanSupplier INTAKE_WALL_SNAPS = FeatureFlag.of("IntakeWallSnaps", true);
  public static final BooleanSupplier INTAKE_DIRECTIONAL_SNAPS =
      FeatureFlag.of("IntakeDirectionalSnaps", false);
  public static final BooleanSupplier DYNAMIC_TURRET_TOLERANCE =
      FeatureFlag.of("DynamicTurretTolerance", true);

  private FeatureFlags() {}
}
