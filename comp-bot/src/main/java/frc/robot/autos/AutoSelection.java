package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.LeftIntegratedAuto;
import frc.robot.autos.auto_state_machines.LeftSpecialAuto;
import frc.robot.autos.auto_state_machines.RightIntegratedAuto;
import frc.robot.autos.auto_state_machines.RightSpecialAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  RIGHT_INTEGRATED_AUTO(RightIntegratedAuto::new),
  RIGHT_SPECIAL_AUTO(RightSpecialAuto::new),
  LEFT_INTEGRATED_AUTO(LeftIntegratedAuto::new),
  LEFT_SPECIAL_AUTO(LeftSpecialAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
