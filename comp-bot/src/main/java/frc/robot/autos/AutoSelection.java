package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.TestAuto;
import frc.robot.autos.auto_state_machines.TrenchMidlineFeedAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  TRENCH_MIDLINE_SHOOT(TrenchMidlineFeedAuto::new),
  TEST(TestAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
