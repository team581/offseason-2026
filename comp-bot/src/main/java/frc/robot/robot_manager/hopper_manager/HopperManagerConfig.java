package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class HopperManagerConfig {
  public static final CANrangeConfiguration CAN_RANGE_CONFIG =
      new CANrangeConfiguration()
          .withFovParams(
              new FovParamsConfigs()
                  .withFOVRangeX(6.75)
                  .withFOVRangeY(6.75)
                  .withFOVCenterY(0.0)
                  .withFOVCenterX(0.0))
          .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.LongRangeUserFreq));
  public static final DoubleSubscriber HOPPER_COMPACTION_DELAY =
      DogLog.tunable("HopperManager/CompactionDelay", 0.7);
}
