package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.LeftBumpAuto;
import frc.robot.autos.auto_state_machines.LeftBumpDepotAuto;
import frc.robot.autos.auto_state_machines.LeftNormalAuto;
import frc.robot.autos.auto_state_machines.RightBumpAuto;
import frc.robot.autos.auto_state_machines.RightNormalAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  LEFT(LeftNormalAuto::new),
  RIGHT(RightNormalAuto::new),
  LEFT_BUMP_CROSS(LeftBumpAuto::new),
  LEFT_BUMP_DEPOT(LeftBumpDepotAuto::new),
  RIGHT_BUMP_CROSS(RightBumpAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
