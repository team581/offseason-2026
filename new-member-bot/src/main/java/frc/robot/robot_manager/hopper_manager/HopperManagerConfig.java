package frc.robot.robot_manager.hopper_manager;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class HopperManagerConfig {
  public static final DoubleSubscriber HOPPER_COMPACTION_DELAY =
      DogLog.tunable("HopperManager/CompactionDelay", 0.25);
  public static final DoubleSubscriber HOPPER_COMPACTION_SHUFFLE_INTERVAL =
      DogLog.tunable("HopperManager/CompactionShuffleInterval", 0.25);
}
