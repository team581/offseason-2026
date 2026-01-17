package frc.robot.config;

import com.team581.config.DSOption;
import com.team581.config.FeatureFlag;

import edu.wpi.first.networktables.BooleanSubscriber;

import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier SHOOT_ON_THE_MOVE = FeatureFlag.of("ShootOnTheMove", false);
public static final BooleanSubscriber VISION_CAMERA_POSITION_COMPENSATION =
      DSOption.of("CameraPositionComp", true);
  public static final BooleanSubscriber VISION_TURRET_POSITION_COMPENSATION =
      DSOption.of("TurretPositionComp", true);
  public static final BooleanSubscriber VISION_TURRET_ANGLE_COMPENSATION =
      DSOption.of("TurretAngleComp", true);
  private FeatureFlags() {}
}
