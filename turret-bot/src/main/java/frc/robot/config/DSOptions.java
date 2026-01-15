package frc.robot.config;

import com.team581.config.DSOption;
import edu.wpi.first.networktables.BooleanSubscriber;

public final class DSOptions {
  public static final BooleanSubscriber SENSOR_BROKEN = DSOption.of("SensorBroken", false);
  public static final BooleanSubscriber VISION_CAMERA_POSITION_COMPENSATION = DSOption.of("CameraPositionComp", true);
  public static final BooleanSubscriber VISION_TURRET_POSITION_COMPENSATION = DSOption.of("TurretPositionComp", true);
  public static final BooleanSubscriber VISION_TURRET_ANGLE_COMPENSATION = DSOption.of("TurretAngleComp", true);

  private DSOptions() {}
}
