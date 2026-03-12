package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.BumpMidlineAuto;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.IntegrationTest;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  BUMP_MIDLINE_AUTO(BumpMidlineAuto::new),
  INTEGRATION_TEST(IntegrationTest::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
